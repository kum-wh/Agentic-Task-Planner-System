# RAG Approaches

Overview of Retrieval-Augmented Generation (RAG) approaches.

## Core pipeline variants
- **Naive RAG**: Chunk, Embed, retrieve top-k by vector similarity, stuff into the prompt. Simple, but weak recall and precision.
- **Advanced RAG**: Adds pre-retrieval steps (query rewriting, query expansion, HyDE, query routing, query decomposition) and post-retrieval steps (reranking).
- **Agentic RAG**: Plans, searches, evaluates, and self-corrects its own information retrieval.

## Retrieval techniques
- **Dense retrieval**: Embedding similarity. Handles semantics well, misses exact terms.
- **Sparse retrieval**: BM25 or TF-IDF keyword matching. Good for IDs, names and jargon.
- **Hybrid search**: Dense plus Sparse, fused with Reciprocal Rank Fusion.
- **Reranking**: A cross-encoder or LLM reranks a wide candidate set down to the top k relevent chunks.
- **Multi-vector** (ColBERT/ColPali): Token-level matching

## Chunking and indexing strategies
- **Fixed-size or recursive chunking**: Baseline Method.
- **Semantic chunking**: Split at topic boundaries.
- **Parent-child / small-to-big**: Retrieve small chunks, feed the larger parent context to the LLM.
- **Contextual retrieval**: Prepend an LLM-generated summary of the document context to each chunk before embedding.
- **Hierarchical indexing / RAPTOR**: Tree of recursive summaries, for questions needing high-level understanding.
- **Multimodal RAG**: Embed images, tables and PDFs, for example ColPali-style page-image retrieval.

## Adaptive and agentic
- **Query routing**: Choose the right index, tool or no retrieval at all.
- **Self-RAG**: Main model decides when to retrieve and critiques its own output.
- **Corrective RAG (CRAG)**: Seperate model grade retrieved docs and fall back to a web search or re-query if poor.
- **Agentic RAG**: An agent treats retrieval as a tool. It plans, issues multiple searches, reflects and decides when it has enough.

## Non-retrieval alternatives
- **Long-context stuffing**: put the whole corpus in the prompt. Suits small, static corpora and cross-document synthesis. Prompt caching helps cost.

## Practical guidance
1. Start with hybrid search, reranking and good chunking and metadata.
2. Add contextual retrieval if recall is poor.
3. Move to agentic or multi-hop retrieval for complex questions.