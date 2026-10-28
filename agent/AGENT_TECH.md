# Architecture for Local Agent

## Overview

```
        ┌────────────────────────────────────────────┐
 user ─▶│  Interface (CLI / UI / API)                │
        └───────────────┬────────────────────────────┘
                        ▼
        ┌────────────────────────────────────────────┐
        │  Agent loop (plan → act → observe → repeat)│
        │   • stop conditions, max steps, retries    │
        └───┬──────────┬──────────┬──────────┬───────┘
            ▼          ▼          ▼          ▼
        Context     Tool       Memory/    Guardrails
        manager     registry   state      & permissions
            │          │          │
            ▼          ▼          ▼
        Local LLM   Local fns   Local store
                        │
                        ▼
                Logging / tracing / evals
```

## Components

### 1. Model Runtime 
- An inference engine such as Ollama or ONNX Runtime serves the model.
- Pick a model that supports **tool/function calling** and has a context window large enough for the tasks.

### 2. Agent loop
- A single loop: build the prompt, call the model, parse a tool call or final answer, execute, append the result, repeat.
- Include a step budget, a timeout, loop and repeat detection, and error handling for malformed output.

### 3. Tool registry
- Tools are ordinary in-process functions. Each has a name, description, and typed argument schema, which are exposed to the model.
- Validate arguments before execution. Return structured results and structured errors.

### 4. Context manager
For models with smaller local context windows, token budgeting for the system prompt, tools, history, and results is essential:
- Truncate or summarize long tool outputs.
- Compact older turns once history nears the limit.
- Load instructions and relevant files on demand instead of all up front.

### 5. Memory and state
- **Short-term:** the conversation and scratchpad in the context window.
- **Persistent:** a local store (JSON files, SQLite, or a markdown notes file) for task state, checkpoints, and facts that should survive restarts.
- **Retrieval:** a local embedding model plus a vector index for documents, if the agent needs to search a knowledge base.

### 6. Guardrails
- Allowlist tools and restrict file paths.
- Confirmation for actions.

### 7. Observability
- Append-only logs of prompt, tool call, and result.
- A small eval set.

Failure modes to design for: bad intermediate output spreading downstream, deadlocks and timeouts leaving inconsistent state, context bloat degrading long runs, and unbounded tool permissions.
