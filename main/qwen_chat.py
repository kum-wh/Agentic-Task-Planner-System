"""Local Qwen3.5 as a LangChain chat model.

Renders the chat with the tool schemas, generates, and parses Qwen3.5's XML tool calls
back into LangChain `tool_calls`.
"""
import json
import re
from pathlib import Path
from uuid import uuid4

import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import AIMessage
from langchain_core.outputs import ChatGeneration, ChatResult
from langchain_core.utils.function_calling import convert_to_openai_tool

MODEL_DIR = Path(__file__).parent / "models" / "Qwen3.5-0.8B"
MODEL_ID = "Qwen/Qwen3.5-0.8B"

_TOOL_RE = re.compile(r"<tool_call>\s*<function=([^>\s]+)>(.*?)</function>\s*</tool_call>", re.S)
_PARAM_RE = re.compile(r"<parameter=([^>\s]+)>\n?(.*?)\n?</parameter>", re.S)
_ROLES = {"system": "system", "human": "user", "ai": "assistant", "tool": "tool"}


def parse_tool_calls(text: str) -> tuple[str, list[dict]]:
    """Split model output into (plain text, tool_calls) for Qwen3.5's XML tool format."""
    calls = []
    for name, body in _TOOL_RE.findall(text):
        args = {}
        for k, v in _PARAM_RE.findall(body):
            v = v.strip()
            try:
                args[k] = json.loads(v)  # numbers, bools; falls back to raw string
            except json.JSONDecodeError:
                args[k] = v
        calls.append({"name": name, "args": args, "id": uuid4().hex[:8], "type": "tool_call"})
    content = _TOOL_RE.sub("", text).split("</think>")[-1].replace("<tool_call>", "").strip()
    return content, calls


def _to_dicts(messages) -> list[dict]:
    """LangChain messages -> Qwen's chat template."""
    out = []
    for m in messages:
        d = {"role": _ROLES[m.type], "content": m.content if isinstance(m.content, str) else str(m.content)}
        if m.type == "ai" and m.tool_calls:
            d["tool_calls"] = [{"function": {"name": c["name"], "arguments": c["args"]}} for c in m.tool_calls]
        out.append(d)
    return out


class ChatQwen(BaseChatModel):
    """Local Qwen3.5 as a LangChain chat model."""

    def __init__(self):
        super().__init__()
        src = str(MODEL_DIR) if MODEL_DIR.exists() else MODEL_ID
        self._tok = AutoTokenizer.from_pretrained(src)
        self._model = AutoModelForCausalLM.from_pretrained(
            src, dtype=torch.bfloat16 if torch.cuda.is_available() else torch.float32,
            device_map="cuda" if torch.cuda.is_available() else "cpu")
        # Define <|im_end|> explicitly;
        self._stop_ids = [self._tok.convert_tokens_to_ids("<|im_end|>"), self._tok.eos_token_id]

    @property
    def _llm_type(self) -> str:
        return "qwen3.5-local"

    def bind_tools(self, tools, **kwargs):
        return self.bind(tools=[convert_to_openai_tool(t) for t in tools], **kwargs)

    def _generate(self, messages, tools=None, **kwargs) -> ChatResult:
        prompt = self._tok.apply_chat_template(
            _to_dicts(messages),
            tools=tools or [],
            add_generation_prompt=True,
            tokenize=False,
            enable_thinking=False,
        )
        inputs = self._tok(prompt, return_tensors="pt").to(self._model.device)
        with torch.inference_mode():
            out = self._model.generate(**inputs, max_new_tokens=1024, do_sample=False,
                                       eos_token_id=self._stop_ids)
        text = self._tok.decode(out[0, inputs["input_ids"].shape[1]:], skip_special_tokens=True)
        content, calls = parse_tool_calls(text)
        # One call per turn: LangChain runs a turn's tool calls in parallel, which breaks
        # order-dependent (stateful) tools. The model sees each result before the next call.
        return ChatResult(generations=[ChatGeneration(message=AIMessage(content=content, tool_calls=calls[:1]))])
