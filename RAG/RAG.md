# RAG retrieval benchmark (ViDoRe V3 Industrial)

## Dataset

[`vidore/vidore_v3_industrial`](https://huggingface.co/datasets/vidore/vidore_v3_industrial) on Hugging Face (US Air Force technical manuals).

| config | what it holds | used for |
|---|---|---|
| `corpus` | one row per page: `corpus_id`, `markdown` (page text), `image`, `doc_id`, `page_number_in_doc` | the pages that are searched |
| `queries` | `query_id`, `query`, `answer`, ... | the questions |
| `qrels` | `query_id`, `corpus_id`, `score`, `bounding_boxes` | ground truth: which pages answer which query |

`qrels.score` is a human relevance grade:

- **2, Fully Relevant:** the page contains the complete answer.
- **1, Critically Relevant:** the page has facts needed for the answer, but other information is required too.

A page with no `qrels` row for a query is not relevant to it. The notebook currently treats scores 1 and 2 the same (binary relevance).

## Pipeline

1. **Load** the `corpus`, `queries` and `qrels` configs.
2. **Index** each page's `markdown` as one document (no chunking), so the thing retrieved is the same thing `qrels` labels.
3. **Retrieve** the top 5 pages per query with each retriever (sparse, dense and visual in `rag.ipynb`).
4. **Score** the results against `qrels` with recall@5 and nDCG@5.

## Results

Run in `rag.ipynb`: 5,244 pages, 1,698 queries with `qrels` labels, `K = 5`, binary relevance.

| retriever | recall@5 | nDCG@5 |
|---|---|---|
| sparse (BM25) | 0.156 | 0.146 |
| dense (Qwen3-Embedding-0.6B) | **0.311** | **0.310** |
| visual (ColSmol-500M) | 0.183 | 0.181 |

- **Dense is the best single retriever,** about twice BM25 on both metrics. Matching meaning beats matching keywords on this set.
- **Visual beats BM25 but not dense.** Page images with the 500M ColPali-style model gain some over keywords, but not enough to beat text embeddings, and they cost far more storage per page (many 128-dim patch vectors instead of one 1024-dim vector).
- **All scores are low in absolute terms.** Even the best retriever finds under a third of a query's relevant pages in the top 5. Part of this is the metric: queries often have many relevant pages (the example query has 10), and recall@5 can never exceed 5 / 10 = 0.5 for it. A larger K, or reading recall relative to `min(K, |relevant|)`, would show more.
- The hybrid (RRF) retriever has no result here. It is not in `rag.ipynb`; see `rag_hybrid.ipynb`.

Other numbers from the run: the page-embedding cell took 1,311 batches (`batch_size=4`), and the visual model produced 128-dim vectors per patch for all 5,244 pages.

## Retrievers

| retriever | how it works | strengths | weaknesses |
|---|---|---|---|
| **Sparse, BM25s** | keyword matching with [`bm25s`](https://github.com/xhluca/bm25s). Text is lowercased, English stopwords are dropped and words are stemmed. Each (word, page) pair gets a weight from how often the word appears in the page, the page's length and how rare the word is across all pages. A page's score is the sum of the weights of the query's words. | exact terms: part numbers, manual codes | misses synonyms and paraphrases |
| **Dense, Qwen3-Embedding-0.6B** | the query and each page are embedded separately into 1024-number vectors. Pages are ranked by cosine similarity to the query. The query gets a task instruction in front of it, and pages are embedded as plain text. | matches meaning, not just words | can miss exact identifiers. Pages beyond 2048 tokens are truncated. Slower than BM25, especially without a GPU. |
| **Visual, ColSmol-500M** | embeds page **images** with [`vidore/colSmol-500M`](https://huggingface.co/vidore/colSmol-500M), a ColPali-style model. Each page becomes a bag of 128-dim patch vectors and each query a bag of token vectors. A page's score is the MaxSim: every query token takes its best-matching patch, and those maxima are summed. | sees layout, tables and figures that text extraction loses | much more storage per page than a single vector. Needs `colpali-engine` and a GPU to be practical. |
| **Hybrid, RRF** (in `rag_hybrid.ipynb`) | takes the top 100 pages from each retriever above and merges them with Reciprocal Rank Fusion. A page scores `1 / (60 + rank)` for each list it appears in, and the scores are added. | uses ranks, so BM25 and cosine scores don't need to be put on one scale | not guaranteed to beat the better single method |

## Metrics

Both metrics are computed per query and then averaged over every query that has `qrels` labels. `K = 5` is set in the benchmark cell.

For each query, `relevant` is its set of correct pages from `qrels`, and `top 5` is the five pages the retriever returned, best first.

### recall@5

> The share of the query's correct pages that appear in the top 5.

```
recall@5 = |relevant ∩ top 5| / |relevant|
```

- If a query has 2 correct pages and 1 is in the top 5, recall@5 is 0.5.
- 1.0 means every correct page was retrieved, 0 means none was.
- It is the most direct measure for RAG, because the LLM only sees the pages you retrieve. A correct page outside the top 5 can never help the answer.
- It ignores the order within the top 5, and it ignores irrelevant pages mixed in with the correct ones.

### nDCG@5

> How well the top 5 is ordered: correct pages ranked higher earn more credit.

**DCG** (discounted cumulative gain) adds up a credit for each correct page in the top 5. The credit is `1 / log2(rank + 1)`, so it shrinks with position:

| rank | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|
| credit | 1.00 | 0.63 | 0.50 | 0.43 | 0.39 |

Incorrect pages add nothing.

**nDCG** (normalised DCG) divides the DCG by the best score possible for that query, which is the DCG if all its correct pages were ranked first. That puts it on a scale from 0 to 1, where 1 means a perfect ordering.

```
nDCG@5 = DCG@5 / ideal DCG@5
```

Example: a query has 2 correct pages.

| retrieved order (✓ = correct) | DCG | ideal DCG | nDCG@5 |
|---|---|---|---|
| ✓ ✓ ✗ ✗ ✗ | 1.00 + 0.63 = 1.63 | 1.63 | 1.00 |
| ✗ ✓ ✓ ✗ ✗ | 0.63 + 0.50 = 1.13 | 1.63 | 0.69 |
| ✗ ✗ ✗ ✗ ✓ | 0.39 | 1.63 | 0.24 |

All three rows have a correct page in the top 5 but different orderings. nDCG rewards the first, and recall@5 would count the second as the same as the first.

### How to read them together

- **recall@5** answers: is the evidence available to the LLM?
- **nDCG@5** answers: is the evidence ranked near the top?

Neither measures the generated answer. Recall can only rise with a larger K, so compare retrievers at the same K, ideally the number of pages you will really feed to the LLM.

### Current limitations

- Relevance is binary. A score-2 page ("full answer") counts the same as a score-1 page ("part of the answer"). Using the grades as gains in nDCG would reward ranking the full-answer page higher.
- Some queries have several score-1 pages that only together give the answer, so an average recall below 1 can mean the LLM is missing part of the evidence.

## Running it

Install the requirements (the notebook has `pip install` cells):

```
datasets pillow pandas bm25s PyStemmer sentence-transformers transformers colpali-engine
```

`transformers` needs to be version 4.51 or newer for Qwen3. `colpali-engine` pins `torch<2.14`, so if pip changes your torch version, restart the kernel before running the visual cells. The notebook also uninstalls `torchao` (Colab's old copy breaks the `transformers` import and nothing here uses it). The first run downloads the dataset (which includes page images), the embedding model (about 1.2 GB) and the ColSmol model. A GPU makes the page-embedding cell much faster. Run the cells from top to bottom. The benchmark cell prints a table with one row per retriever (sparse, dense, visual) and the columns `recall@5` and `nDCG@5`.

## Settings you may want to change

| setting | where | default |
|---|---|---|
| cutoff `K` | benchmark cell | 5 |
| embedding model `EMBED_MODEL` | dense retrieval cell | `Qwen/Qwen3-Embedding-0.6B` |
| query instruction `QUERY_PROMPT` | dense retrieval cell | "Given a question about a technical manual, retrieve the page that answers it" |
| max page length `max_seq_length` | dense retrieval cell | 2048 tokens |
| page-embedding batch size | dense retrieval cell | 4 |
| visual model `VISUAL_MODEL` | visual retrieval cell | `vidore/colSmol-500M` (swap in `colSmol-256M` for speed or `colqwen2.5-v0.2` for accuracy) |
| `PAGE_BATCH`, `SCORE_BATCH` | visual retrieval cell | 8 and 64 |
| RRF constant `rrf_k` and `depth` | `hybrid_pages` in `rag_hybrid.ipynb` | 60 and 100 |
