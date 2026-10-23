# Drone-fleet agent (LangGraph + Qwen3.5-0.8B)

`agent.ipynb` builds a small tool-calling agent that runs fully locally. A LangGraph agent drives [`Qwen/Qwen3.5-0.8B`](https://huggingface.co/Qwen/Qwen3.5-0.8B), wrapped as a LangChain chat model. It has four tools: list the fleet, plan a search path, send a drone to a point, and search operations documents.

The drone tools are **mocks**. They return canned strings and don't control anything.

## Architecture

```
user prompt
    |
    v
create_agent (LangGraph)  <-- system prompt, InMemorySaver (per-thread memory)
    |        ^
    v        | tool results
ChatQwen  --------> tools: list_drones, plan_path, navigate, search_docs
(local Qwen3.5-0.8B)                                            |
                                                  InMemoryVectorStore (docs/*.md, *.txt)
```

The loop runs until the model replies without a tool call, or until the step limit is hit.

## Model wrapper (`ChatQwen`)

Qwen3.5 is not served through an API here, so `ChatQwen` subclasses `BaseChatModel` and does three jobs:

| step | what it does |
|---|---|
| **Render** | `_to_dicts` converts LangChain messages into Qwen's chat roles. `apply_chat_template` renders them together with the tool schemas (`bind_tools` converts each tool with `convert_to_openai_tool`). Thinking is off (`enable_thinking=False`). |
| **Generate** | Greedy decoding (`do_sample=False`), `max_new_tokens=1024`. Stops on `<|im_end|>` or EOS, with `<|im_end|>` set explicitly. |
| **Parse** | `parse_tool_calls` reads Qwen's XML format, `<tool_call><function=name><parameter=k>v</parameter></function></tool_call>`, into LangChain `tool_calls`. Parameter values go through `json.loads`, so numbers and bools are typed, and anything else stays a string. Any `<think>` text before `</think>` is dropped from the content. |

**One tool call per turn.** The wrapper keeps only `calls[:1]`. LangChain runs a turn's tool calls in parallel, which would break order-dependent tools such as `navigate`. With one call per turn, the model sees each result before it makes the next call.

The model loads from `./models/Qwen3.5-0.8B` if that folder exists, otherwise it downloads from the Hub. It uses bfloat16 on CUDA and float32 on CPU.

## Tools

| tool | purpose | notes |
|---|---|---|
| `list_drones()` | lists each drone's id, state, position and battery | Mock data: `alpha-1` (idle, 92%), `alpha-2` (searching, 61%), `bravo-1` (returning, 18%). The docstring says drones under 25% battery should not get new searches. |
| `plan_path(latitude, longitude, radius_m, pattern)` | plans waypoints that cover a circular region | Plans only, doesn't move a drone. `pattern` is `lawnmower` (default) or `spiral`. Returns an error string for an unknown pattern, out-of-range coordinates or a non-positive radius. |
| `navigate(drone_id, latitude, longitude, altitude_m)` | sends one drone to one point | Called once per waypoint. `altitude_m` defaults to 60 m. Returns a canned "ok ... ETA 4 min". |
| `search_docs(query)` | semantic search over the operations documents | Returns the top 3 chunks with their source file. See [Documents (RAG)](#documents-rag). |

Tool errors are returned as strings, so the model can read them and adapt. Each docstring says when to use the tool, because the small model picks tools from those descriptions.

### Path planning

`plan_path` works in metres east/north of the centre, then converts to latitude and longitude.

- **Sweep spacing** is `radius_m / 3` for both patterns.
- **Lawnmower** (`_lawnmower_xy`): east-west lines, clipped to the circle, alternating direction.
- **Spiral** (`_spiral_xy`): an Archimedean spiral from the centre outwards, 6 points per turn, ending at the radius.
- **Conversion:** 111,320 m per degree of latitude. Longitude is scaled by `cos(latitude)`.
- **Estimates:** total length is the sum of the segments. Flight time assumes a cruise speed of 5 m/s.

The result lists numbered waypoints along with the waypoint count, total length and flight time.

`draw_path` / `show_path` plot the path over the search circle (green = start, red = end). They are **notebook-only**. In deployment, delete that cell and the `show_path(...)` call in `plan_path`, and nothing else changes.

## Documents (RAG)

The notebook indexes the files in `docs/` once, in memory, and exposes them as the `search_docs` tool. The agent decides when to call it.

| setting | value |
|---|---|
| source | `docs/*.md` and `docs/*.txt` |
| splitter | `RecursiveCharacterTextSplitter`, `chunk_size=500`, `chunk_overlap=50` |
| embeddings | `sentence-transformers/all-MiniLM-L6-v2` |
| store | `InMemoryVectorStore` (rebuilt on every run) |
| retrieval | `similarity_search`, `k=3` |

If `docs/` has no matching files, the store stays empty and the tool returns "no matching documents found". See [../RAG/RAG.md](../RAG/RAG.md) for the retriever benchmark.

## Agent

```python
agent = create_agent(model=llm, tools=list_tools(),
                     system_prompt=SYSTEM, checkpointer=InMemorySaver())
```

The system prompt tells the model to:

- call one tool at a time, in order, and never invent tools or arguments;
- read tool errors and adapt, or tell the user why it can't do the request;
- answer policy questions (procedures, limits, rules) only from what `search_docs` returns;
- reply briefly once it has the answer, with no more tool calls.

`MAX_STEPS = 12` bounds a run through `recursion_limit = 2 * MAX_STEPS + 1`. When the limit is hit, `ask()` prints "Step limit reached".

## Chat

`ask(text, thread_id="1")` sends one prompt and streams the reply tokens and each tool call, with its input and result, to the console. Turns that share a `thread_id` share conversation memory, because `InMemorySaver` is keyed by thread. Pass a new `thread_id` to start a fresh conversation. Memory is lost when the kernel restarts.

The notebook ends with two example prompts and an interactive loop (submit an empty prompt to stop):

```python
ask("Which drones are free right now?")
ask("Plan a spiral search with a 300 m radius around 1.3521, 103.8198 and send the free drone to its centre.")
```

## Running it

Install the requirements (the first cell has the `pip install`):

```
langchain langgraph langchain-community "langchain[huggingface]" sentence-transformers langchain-text-splitters
```

The notebook also needs `torch`, `transformers` and `pillow`, and `langchain-huggingface` for the embeddings. A GPU is optional. The 0.8B model runs on CPU, more slowly.

1. Put the operations documents (`.md` or `.txt`) in `docs/` next to the notebook.
2. Run the cells **from top to bottom.** `list_tools()` refers to `search_docs`, which is defined in the *Documents (RAG)* cell *after* the Tools cell. Call it (in the Agent cell) only after both cells have run, or it raises `NameError`.
3. Use `ask(...)` or the interactive loop.

## Settings you may want to change

| setting | where | default |
|---|---|---|
| model path / id `MODEL_DIR`, `MODEL_ID` | model wrapper cell | `./models/Qwen3.5-0.8B`, `Qwen/Qwen3.5-0.8B` |
| `max_new_tokens` | `ChatQwen._generate` | 1024 |
| tool calls kept per turn | `ChatQwen._generate` (`calls[:1]`) | 1 |
| `CRUISE_MPS` | tools cell | 5.0 m/s |
| sweep spacing | `plan_path` | `radius_m / 3` |
| `VALID_PATTERNS` | tools cell | `lawnmower`, `spiral` |
| `DOCS_DIR` | RAG cell | `docs` |
| chunk size / overlap | RAG cell | 500 / 50 |
| embedding model | RAG cell | `sentence-transformers/all-MiniLM-L6-v2` |
| `k` in `search_docs` | RAG cell | 3 |
| `MAX_STEPS` | Agent cell | 12 |
| `SYSTEM` prompt | Agent cell | see [Agent](#agent) |

## Current limitations

- All drone tools are mocks: `list_drones` is static and `navigate` doesn't change any state, so the fleet never updates.
- The agent doesn't enforce the 25% battery rule or the altitude range (10 to 120 m). The model has to follow the docstrings and the retrieved documents.
- Flying a path means one `navigate` call per waypoint. A lawnmower over a large region can use up `MAX_STEPS` before the last waypoint.
- `ChatQwen` accepts only one tool call per turn, which is slower but safe for stateful tools.
- A 0.8B model can pick the wrong tool or arguments. Check the `Tool call:` lines in the output.
