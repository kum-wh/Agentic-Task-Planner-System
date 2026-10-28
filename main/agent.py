"""LangGraph agent using a local Qwen3.5-0.8B.

Run `python agent.py` for an interactive chat (empty prompt to stop), or
`python agent.py "your prompt"` for a single question.

Requirements: langchain langgraph langchain-community "langchain[huggingface]"
langchain-huggingface sentence-transformers langchain-text-splitters torch transformers
"""
import sys

from langchain.agents import create_agent
from langgraph.checkpoint.memory import InMemorySaver
from langgraph.errors import GraphRecursionError

# from docs_rag import build_index, search_docs
from tools import list_drones, navigate, plan_path
from qwen_chat import ChatQwen

MAX_STEPS = 12

SYSTEM = """You are a helpful assistant with access to tools. Fulfil the user's request by calling tools when needed.
Rules:
- Call one tool at a time, in order. Never invent tools or arguments.
- If a tool returns an error, read it and adapt, or tell the user why you cannot do it.
- For questions about procedures, limits or rules, call search_docs and answer only from what it returns.
- When you have the answer, reply briefly. Do not call more tools."""


def list_tools() -> list:
    return [plan_path, list_drones, navigate]

def ask(agent, text: str, thread_id: str = "1") -> None:
    """Send one prompt and stream the replies and tool calls.

    Turns that share a `thread_id` share conversation memory; pass a new
    `thread_id` to start a fresh conversation.
    """
    config = {"configurable": {"thread_id": thread_id}, "recursion_limit": 2 * MAX_STEPS + 1}
    try:
        stream = agent.stream_events(
            {"messages": [{"role": "user", "content": text}]},
            config=config,
            version="v3",
        )
        for kind, item in stream.interleave("messages", "tool_calls"):
            if kind == "messages":
                for token in item.text:
                    print(token, end="", flush=True)
            elif kind == "tool_calls":
                print(f"\nTool call: {item.tool_name}({item.input})")
                for delta in item.output_deltas:
                    print(delta, end="", flush=True)
                print(f"\nTool result: {item.output}")
        print()
    except GraphRecursionError:
        print("\nAgent> Step limit reached.")
    except Exception as e:
        print(f"\nAgent> Error: {type(e).__name__}: {e}")


def main():
    # build_index()
    agent = create_agent(model=ChatQwen(), tools=list_tools(), system_prompt=SYSTEM, checkpointer=InMemorySaver()) 
    if len(sys.argv) > 1:
        ask(agent, " ".join(sys.argv[1:]))
        return
    while text := input("Your prompt: ").strip():
        ask(agent, text)


if __name__ == "__main__":
    main()
