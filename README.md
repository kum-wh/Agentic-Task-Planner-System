# Agentic System — Architecture

The architecture for an LLM agent that takes a goal in natural language and completes it
by planning, calling tools, checking its own work and asking a human when it should. The
design is task-agnostic: a task is defined by its **tools**, **validators** and **policies**,
not by changes to the core.

> **Status: design.** This document describes the target architecture. The current prototype
> in [agent/agent.md](agent/agent.md) is a single tool-calling loop; retrieval experiments are
> in [RAG/RAG.md](RAG/RAG.md).

---

## 1. Principles

1. **The model proposes, deterministic code disposes.** The LLM chooses actions. Validation,
   permissions and anything with side effects are enforced by code the model cannot edit.
2. **Structure over prose.** Plans, tool calls and verdicts are typed objects, produced with
   schema-constrained decoding, not parsed out of free text.
3. **Least privilege per step.** Each step sees only the tools and context it needs.
4. **Verify, don't trust.** Every step's output passes a checker before the next step builds on
   it. Cheap deterministic checks run first; an LLM judge only covers what code cannot.
5. **Bounded and resumable.** Loops have budgets. State is checkpointed, so a run can pause for
   a human, crash and resume, or replay.
6. **Observable by default.** Every model call, tool call and state change is traced and can
   be scored offline.

---

## 2. System overview

```
                         ┌───────────────────────────────────────────────┐
   user / API ─────────▶ │ Orchestrator (durable state machine)          │
                         │   plan ─▶ execute ─▶ verify ─▶ reflect ─┐     │
                         │     ▲                                   │     │
                         │     └───────────── replan ◀─────────────┘     │
                         │   budgets · checkpoints · interrupts          │
                         └───┬─────────┬──────────┬──────────┬───────────┘
                             │         │          │          │
                      ┌──────▼───┐ ┌───▼────┐ ┌───▼─────┐ ┌──▼────────┐
                      │ Model    │ │ Tool   │ │ Memory  │ │ Retrieval │
                      │ layer    │ │ layer  │ │ & ctx   │ │ (RAG)     │
                      └──────────┘ └───┬────┘ └─────────┘ └───────────┘
                                       │
                               ┌───────▼────────┐
                               │ Policy engine  │ ◀── human approval
                               │ + sandbox      │
                               └────────────────┘

                  Tracing · evals · replay  (cross-cutting, every box above)
```

| Layer | Responsibility |
|---|---|
| **Orchestrator** | Runs the control loop as an explicit graph; owns state, budgets, checkpoints, human interrupts |
| **Model layer** | Model routing, structured output, prompt caching, retries |
| **Tool layer** | Typed tool registry, capability tiers, execution in a sandbox |
| **Policy engine** | Allow / deny / require-approval decisions, independent of the model |
| **Memory & context** | Working state, episodic and semantic memory, context assembly and compaction |
| **Retrieval** | Hybrid search with reranking and citations, exposed as a tool |
| **Observability** | Traces, metrics, trajectory evals, replay |

---

## 3. Orchestrator

The control loop is a **graph with explicit nodes and edges**, not an unbounded ReAct loop.
Open-ended loops are used only *inside* a step, where flexibility pays for itself.

```
  goal ─▶ [ clarify ] ─▶ [ plan ] ─▶ [ execute step ] ─▶ [ verify ] ─┬─ pass ─▶ next step ─▶ … ─▶ [ finalize ]
              │             ▲              │                         │
         ask the user       │              └── tool loop (ReAct)     └─ fail ─▶ [ reflect ] ─┬─ retry step
                            │                                                              ├─ replan ─┘
                            └──────────────────────────────────────────────────────────────┴─ escalate to human
```

| Node | Role |
|---|---|
| **clarify** | Resolve missing or ambiguous inputs with at most one question |
| **plan** | Produce a typed plan (section 4) |
| **execute step** | Run one plan step: a bounded tool-use loop (ReAct style) with only that step's tools |
| **verify** | Check the step's output against its acceptance criteria (section 7) |
| **reflect** | On failure, diagnose and choose: retry the step, replan, or escalate |
| **finalize** | Assemble the result with citations and a summary of what was done |

**Budgets** bound every loop: max steps, max retries per step, max replans, token and wall-clock
limits, and a cost cap. Hitting a budget is a first-class outcome (escalate or stop with a
reason), not an exception.

**Multi-agent, when it earns its place.** The default is one orchestrator with step-scoped
tool sets. A step may spawn a **worker agent** (orchestrator–worker pattern) when work is
parallel or context-heavy, for example research over many documents. Workers get a narrow brief
and a clean context, return a compact structured result, and never share raw transcripts.

---

## 4. Planning

**Plan-and-execute with replanning.** A planner model writes the whole plan up front; an
executor runs it step by step; the plan is revised when verification fails or new information
arrives.

A plan is a typed DAG, validated before execution:

```
Plan
  goal: str
  steps: [
    Step(
      id, description,
      tools: [tool names allowed for this step],
      depends_on: [step ids],
      inputs: { name: literal | ref(step_id.output) },
      acceptance: [Check]            # what "done" means, checkable
      risk: read | write | irreversible
    )
  ]
```

- **Validated before it runs.** Unknown tools, dependency cycles, references to missing
  outputs and steps that ask for tools outside the task's allow-list are rejected, and the
  planner is told why.
- **Acceptance criteria are part of the plan.** The planner states how each step will be
  checked, which makes verification concrete and gives the evals something to score.
- **Independent steps run in parallel.** The DAG makes this explicit.
- **Replan from state, not from scratch.** Completed steps and their verified outputs are kept;
  only the failed branch is regenerated.
- **Reasoning budget follows difficulty.** Planning and reflection use a stronger model or a
  higher reasoning effort; routine tool steps use a cheaper one (section 8).

---

## 5. Tool layer

Every capability is a **tool with a typed contract**, registered in one place.

```
Tool
  name, description               # description says when to use it, and when not to
  input_schema, output_schema     # JSON Schema; validated on every call
  tier: read | write | irreversible
  idempotent: bool
  timeout, rate_limit
```

- **Open standard for integration.** Tools are exposed over the Model Context Protocol (MCP),
  so tool servers can be added or swapped without touching the orchestrator.
- **Schema-constrained calls.** Arguments are generated with constrained decoding against the
  tool's schema and validated again on receipt. Invalid calls return a short, actionable error
  the model can correct from.
- **Capability tiers.** `read` tools run freely. `write` tools are logged and, by policy, may
  need approval. `irreversible` tools always go through the policy engine and, by default, a
  human.
- **Idempotency and dry-run.** Write tools take an idempotency key so a retried step cannot
  apply twice, and expose a dry-run mode where possible so the plan can be previewed.
- **Programmatic tool use.** For multi-call data wrangling, the model may write a short script
  that calls tools inside the sandbox, so intermediate results stay out of the context window.
- **Sandboxed execution.** Code and shell tools run in an isolated environment with no ambient
  credentials, restricted network egress and resource limits.
- **Tool selection at scale.** With many tools, a retrieval step narrows the registry to the
  few relevant to the current step instead of loading every schema.

---

## 6. Memory and context engineering

Context is assembled deliberately for each model call rather than appended to forever.

| Memory | Contents | Lifetime | Mechanism |
|---|---|---|---|
| **Working** | Current plan, step outputs, scratch notes | One run | Structured state in the graph, checkpointed |
| **Episodic** | Past runs: goal, plan, outcome, what failed | Across runs | Stored traces, retrieved by similarity to the new goal |
| **Semantic** | Facts, preferences, domain knowledge | Long-lived | Vector store with provenance and timestamps |
| **Procedural** | Reusable plans, prompts, tool recipes | Long-lived | Versioned library, promoted from successful runs |

**Context assembly** builds each prompt from labelled sections: stable instructions first, then
the step brief, relevant memory, the retrieved evidence, and recent tool results.

- **Stable prefix, volatile suffix.** Instructions and tool schemas come first and stay
  byte-identical between calls, so **prompt caching** applies and cost and latency drop.
- **Compaction.** Long transcripts are summarised into structured state at step boundaries.
  Large tool outputs are stored by reference and shown truncated, with a handle to fetch more.
- **Just-in-time retrieval.** Memory and documents are pulled when a step needs them instead of
  preloaded.
- **Write policy for memory.** Only verified facts and successful plans are promoted, with the
  run that produced them. Memory entries from untrusted sources are marked as such.

---

## 7. Verification and reflection

Quality comes from checking work, not from a better prompt.

**Verification, cheapest first:**

1. **Schema check.** Does the output parse and match its type?
2. **Deterministic validators.** Task-specific code: invariants, constraints, unit-style checks,
   cross-checks against a source of truth. These are authoritative; a pass here outranks an
   LLM's opinion.
3. **Grounding check.** Each claim in a generated answer must trace to a retrieved source or
   tool result; unsupported claims are flagged.
4. **LLM judge.** For what code cannot express (relevance, completeness, tone), a separate
   judge model scores the output against the step's acceptance criteria. The judge sees the
   criteria and the output, not the producer's reasoning, and is a different prompt (ideally a
   different model) from the producer to reduce self-agreement.

**Reflection** runs only after a failed check. It receives the failure and the evidence, not the
whole history, and returns a typed decision:

```
Reflection { diagnosis, action: retry_step | replan | escalate, change: <what is different next time> }
```

A retry must differ from the previous attempt (changed arguments, tool or approach). The same
failing attempt twice in a row escalates. Reflections are stored in episodic memory so the same
failure is avoided on the next run.

**Best-of-n and self-consistency** are optional for steps where one wrong answer is costly:
sample several candidates, verify each, take the one that passes.

---

## 8. Model layer

- **Routing.** A small fast model handles routing, extraction and simple tool steps. A stronger
  model, with extended reasoning where it helps, handles planning, reflection and judging.
  Routing is a config table keyed by node, not logic scattered through the code.
- **Structured outputs everywhere** the result feeds code: plans, tool arguments, verdicts,
  reflections.
- **Prompt caching** on the stable prefix (section 6); **streaming** for the user-facing
  answer; **retries with backoff** and fallback models on provider errors.
- **Provider-agnostic interface.** One `generate(messages, schema, tools)` boundary, so models
  can be swapped, including local models, without changing nodes.
- **Prompts are versioned artifacts**, tested like code against the eval suite (section 11).

---

## 9. Retrieval

Retrieval is a tool the agent decides to use, and a pipeline behind it.

```
query ─▶ rewrite / decompose ─▶ ┬─ sparse (BM25) ─┐
                                └─ dense (embed)  ┴─▶ fuse (RRF) ─▶ rerank (cross-encoder) ─▶ top-k with citations
```

- **Hybrid search.** Sparse retrieval catches exact identifiers; dense retrieval catches
  paraphrase. Results are merged with reciprocal rank fusion.
- **Reranking.** A cross-encoder reranks the fused candidates before they reach the model.
- **Contextual chunks.** Each chunk is stored with a short document-level context so it is
  retrievable on its own; page-structured documents are indexed per page where that matches how
  answers are labelled.
- **Agentic RAG.** The agent can rewrite the query, split a multi-part question, retrieve again
  after seeing results, and stop when the evidence is enough. A retrieval budget bounds this.
- **Citations are mandatory.** Every retrieved chunk carries source and location, and the
  grounding check (section 7) uses them.
- **Measured, not assumed.** Retrievers are benchmarked on labelled queries (recall@k, nDCG@k)
  before one is chosen; see [RAG/RAG.md](RAG/RAG.md) for the harness and results.

---

## 10. Safety, policy and security

### Policy engine

A rules layer between the model and anything with side effects. It is code, configured per
task, and the model has no tool that reads or edits it.

```
decide(tool, args, run_context) -> allow | deny(reason) | require_approval(reason)
```

Rules typically cover tool tier, argument constraints (allow-listed targets, value ranges),
rate and spend limits, and required prior steps (for example "no write before the dry-run
passed"). A denial returns to the agent as a readable reason.

### Human-in-the-loop

- **Interrupts.** The orchestrator can pause at any node, persist state, and wait for a human.
  Approval is bound to the exact action: the approval records a hash of the tool name and
  arguments, and execution refuses if either changed.
- **Escalation, not guessing.** Exhausted budgets, repeated failures, low-confidence judges and
  policy conflicts hand off to a human with a compact summary of the evidence.
- **Approvals are minted by the interface**, never by a model call.

### Prompt-injection and untrusted data

- **Trust labels.** Every piece of context is tagged `trusted` (system, user) or `untrusted`
  (retrieved documents, web pages, tool outputs, files). Untrusted text is delimited and never
  placed in the instruction channel.
- **Capability separation.** A step that reads untrusted content gets no write or irreversible
  tools. Where both are needed, use a **dual-model pattern**: a quarantined model processes the
  untrusted content and returns only typed, validated fields; the privileged model plans and acts
  on those fields without seeing the raw text.
- **Egress control.** Outbound network and data-leaving tools are allow-listed.
- **Secrets stay out of context.** Tools hold credentials; the model never sees them.

### Sandboxing

Anything that executes code or touches the filesystem runs in an isolated, ephemeral
environment with explicit mounts, no ambient credentials and resource limits.

---

## 11. Observability and evaluation

- **Tracing.** Every run is a trace of spans: model calls (prompt, output, tokens, cost,
  latency), tool calls (args, result, duration), state transitions, verifier verdicts, human
  decisions. Traces are the unit of debugging and of replay.
- **Replay.** A run can be re-executed from its trace with recorded tool results, to reproduce a
  failure or test a prompt change without touching live systems.
- **Offline evals.** A versioned scenario set covers normal tasks, ambiguous requests, missing
  data, tool failures, and adversarial inputs including injection attempts. Scored on:

  | Metric | Question |
  |---|---|
  | task success | Did the final result meet the acceptance criteria? |
  | trajectory quality | Right tools, right order, no wasted steps? |
  | verifier agreement | Does the LLM judge agree with deterministic checks and human labels? |
  | safety | Any policy violation or unapproved side effect? Must be zero. |
  | efficiency | Steps, tokens, cost, latency per task |

- **Judge calibration.** LLM judges are checked against a human-labelled sample before being
  trusted, and re-checked when the judge model or prompt changes.
- **Online monitoring.** The same metrics run on sampled production traces, with alerts on
  budget exhaustion rate, escalation rate, policy denials and cost drift.
- **Regression gate.** Prompt, model, tool and policy changes must pass the eval suite before
  release.

---

## 12. Durability

- **Checkpoint after every node.** State is persisted so a run survives crashes and deploys,
  and can wait indefinitely for a human.
- **Resume, don't restart.** Execution continues from the last checkpoint with completed steps
  intact. Write tools use idempotency keys so a replayed step cannot double-apply.
- **Time travel.** Any checkpoint can be inspected, edited and forked, which is how a human
  corrects an agent mid-run.
- **Concurrency.** Runs are isolated; shared resources are guarded by the tool layer, not by
  the model's good behaviour.

---

## 13. Defining a task

The core stays fixed. A new task is a package of:

| Piece | Provides |
|---|---|
| **Tools** | Typed capabilities, with tiers |
| **Validators** | Deterministic checks for step outputs and the final result |
| **Policies** | Allow / deny / approval rules |
| **Knowledge** | Documents indexed for retrieval |
| **Prompts** | Role and step instructions, versioned |
| **Eval scenarios** | What good looks like, including failures |

---

## 14. Layout

```
README.md      this document
agent/         orchestrator, tools, memory (prototype today)
RAG/           retrieval benchmark and retrievers
```
