"""Indexes the files in `docs/` once, in memory, and exposes them as the `search_docs` tool."""
from pathlib import Path

from langchain_core.tools import tool
from langchain_core.vectorstores import InMemoryVectorStore
from langchain_huggingface import HuggingFaceEmbeddings
from langchain_text_splitters import RecursiveCharacterTextSplitter

DOCS_DIR = Path(__file__).parent / "docs"

_store: InMemoryVectorStore | None = None


def build_index() -> InMemoryVectorStore:
    """Embed `docs/*.md` and `docs/*.txt` into the in-memory store (built once, reused after)."""
    global _store
    if _store is not None:
        return _store

    embeddings = HuggingFaceEmbeddings(model_name="sentence-transformers/all-MiniLM-L6-v2")
    splitter = RecursiveCharacterTextSplitter(chunk_size=500, chunk_overlap=50)

    files = [p for p in sorted(DOCS_DIR.glob("*")) if p.suffix in (".md", ".txt")]
    chunks = splitter.create_documents(
        [p.read_text(encoding="utf-8") for p in files],
        metadatas=[{"source": p.name} for p in files],
    )
    _store = InMemoryVectorStore(embeddings)
    if chunks:
        _store.add_documents(chunks)
    print(f"indexed {len(chunks)} chunks from {len(files)} files in {DOCS_DIR.resolve()}")
    return _store


@tool
def search_docs(query: str) -> str:
    """Search the operations documents for procedures, limits and rules.

    Use this when a request depends on documented policy, e.g. battery
    minimums, altitude limits or flight rules. It only reads documents;
    it does not move any drone.

    Args:
        query: What to look for, in a few words, e.g. 'minimum battery for a search'.
    """
    hits = build_index().similarity_search(query, k=3)
    if not hits:
        return "no matching documents found"
    return "\n\n".join(f"[{h.metadata.get('source', '?')}]\n{h.page_content}" for h in hits)
