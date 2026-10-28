# RAG retrieval benchmark (ViDoRe V3 Industrial)

## Dataset

[`vidore/vidore_v3_industrial`](https://huggingface.co/datasets/vidore/vidore_v3_industrial) on Hugging Face (US Air Force technical manuals).

| config | what it holds | used for |
|---|---|---|
| `corpus` | one row per page: `corpus_id`, `markdown` (page text), `image`, `doc_id`, `page_number_in_doc` | the pages that are searched |
| `queries` | `query_id`, `query`, `answer`, ... | the questions |
| `qrels` | `query_id`, `corpus_id`, `score`, `bounding_boxes` | ground truth: which pages answer which query |

`qrels.score` is a relevance grade:

- **2, Fully Relevant:** the page contains the complete answer.
- **1, Critically Relevant:** the page has facts needed for the answer, but other information is required too.

A page with no `qrels` row for a query is not relevant to it. The notebook currently treats scores 1 and 2 the same (binary relevance).

The dataset has 1,698 queries, but these are 283 distinct questions each written in 6 languages (every query has the same relevant pages in each language). The corpus is English, so the notebooks keep only the rows with `language == "english"`: **283 queries and 1,614 `qrels` rows**.

## Pipeline

1. **Load** the `corpus`, `queries` and `qrels` configs, and keep only the English queries.
2. **Index** each page's `markdown` as one document (no chunking), so the thing retrieved is the same thing `qrels` labels.
3. **Retrieve** the top 5 pages per query with each retriever (sparse, dense and hybrid in `rag_text.ipynb`; visual and fused sparse + dense + visual in `rag_image.ipynb`).
4. **Score** the results against `qrels` with recall@5 and nDCG@5.

## Results

### Text retrievers, English queries only (`rag_text.ipynb`)

5,244 pages, 283 English queries with `qrels` labels, `K = 5`, binary relevance.

| retriever | recall@5 | nDCG@5 |
|---|---|---|
| sparse (BM25) | 0.426 | 0.437 |
| dense (Qwen3-Embedding-0.6B) | 0.400 | 0.419 |
| hybrid (RRF, pool 50, c = 60) | **0.433** | **0.448** |

- **All three are within about 3 points of each other.** With 283 queries the 95% interval on one score is roughly ±5 points, so these results do not show a clear winner. Hybrid being nominally first is not evidence that it is better.
- **Filtering to English is what changed the picture.** In the all-language run (below) BM25 scored 0.156 and dense 0.311. BM25 uses an English stemmer and stopword list, and 5 of 6 queries there were non-English, so it was matching almost nothing. On English alone BM25 is competitive with dense, and hybrid is no longer worse than dense.
- **The two retrievers find different queries.** Both hit 164 queries, BM25 alone hit 27, dense alone hit 35, and 57 queries (20%) were missed by both. BM25 misses 92 of 283 queries (recall@5 = 0) and dense misses 84. Hybrid misses about 77.
- **BM25 has the higher recall@5 but the higher miss rate.** A likely reason is that when BM25 hits, it returns more of the several relevant pages from the same manual, but this was not checked.

Miss rate (share of queries with recall@5 = 0, lower is better) by query attribute, English only:

| group | BM25 | dense | hybrid |
|---|---|---|---|
| keyword queries (n=38) | 0.289 | **0.132** | 0.184 |
| question queries (n=161) | 0.286 | 0.298 | **0.242** |
| instruction queries (n=84) | 0.417 | **0.369** | **0.369** |
| open-ended (n=67) | 0.687 | **0.493** | 0.522 |
| extractive (n=125) | 0.192 | 0.232 | **0.176** |
| numerical (n=29) | **0.069** | 0.103 | 0.103 |
| content: image (n=21) | 0.476 | **0.286** | 0.381 |
| content: infographic (n=54) | 0.407 | **0.241** | 0.259 |
| content: table (n=88) | 0.375 | 0.307 | **0.273** |
| content: text (n=273) | 0.330 | 0.297 | **0.271** |

- **Dense is better on keyword queries,** the opposite of what keyword matching would suggest. BM25 missed 11 of the 38 keyword queries and dense found 7 of those 11.
- **BM25 is weakest on figure content.** Image and infographic queries have its highest miss rates (0.48 and 0.41), and dense still misses about a quarter of them. The page-image retrievers do not clearly fix this; see the visual results below.
- **BM25 is closest to dense on exact-term questions** (extractive, numerical), which fits how it matches words.
- Query attributes are multi-label, so the `n` values add up to more than 283 for content type and query type. Groups with fewer than about 20 queries (chart, "other", "N/A") are too small to read.

Typical BM25 misses are vocabulary mismatch: for "factors affecting aircraft servicing safety zones" the relevant page contained only "aircraft", "safety" and "servicing" among the query's stemmed terms.

### Visual retrievers and fusion, English queries only (`rag_image.ipynb`)

Same 5,244 pages, 283 English queries, binary relevance. The text rows are rerun in this notebook, so they can differ from the `rag_text.ipynb` tables by about one query. The highest recall any retriever could reach is 0.828 at K = 5, 0.934 at K = 10 and 0.992 at K = 20, because queries with many relevant pages cannot reach 1.0 at a small K.

| retriever | recall@5 | nDCG@5 | recall@10 | nDCG@10 | recall@20 | nDCG@20 |
|---|---|---|---|---|---|---|
| sparse (BM25) | 0.426 | 0.437 | 0.504 | 0.451 | 0.583 | 0.477 |
| dense (Qwen3-Embedding-0.6B) | 0.400 | 0.420 | 0.460 | 0.427 | 0.549 | 0.455 |
| visual (ColQwen2.5, multi-vector) | 0.468 | 0.509 | 0.552 | 0.525 | 0.636 | **0.551** |
| Qwen3-VL-Embedding-2B, page image | 0.426 | 0.434 | 0.503 | 0.447 | 0.628 | 0.487 |
| Qwen3-VL-Embedding-2B, page image + markdown | 0.434 | 0.447 | 0.533 | 0.470 | 0.621 | 0.498 |
| text hybrid (sparse + dense, RRF) | 0.437 | 0.450 | 0.525 | 0.468 | 0.597 | 0.492 |
| fusion: sparse + dense + ColQwen2.5, equal weights | 0.470 | 0.495 | 0.558 | 0.513 | **0.647** | 0.543 |
| fusion: sparse + dense + 3x ColQwen2.5 | 0.480 | 0.504 | 0.557 | 0.520 | 0.638 | 0.546 |
| fusion: sparse + dense + 5x ColQwen2.5 | **0.484** | **0.511** | **0.570** | **0.528** | 0.640 | **0.551** |

- **ColQwen2.5 is the best single retriever.** At K = 5 it is 4.2 points of recall above the best text retriever (BM25) and 3.1 above text hybrid. In nDCG it is 5.9 points above text hybrid. The lead holds at K = 10 and 20. It is at the edge of the roughly ±5 point interval at 283 queries and has no paired significance test.
- **It replaced ColSmol-500M, which scored 0.425 recall@5 and 0.436 nDCG@5.** The gain is 4.3 points of recall and 7.3 of nDCG. It came on compare-contrast (miss rate 0.207 down to 0.103), extractive (0.192 to 0.120) and numerical queries (0.103 to 0). Tables did not improve (0.250 for both).
- **Equal-weight RRF does not beat ColQwen2.5 alone.** Recall is the same (0.470 against 0.468) and nDCG is lower (0.495 against 0.509), because the two text lists outvote the one visual list.
- **Weighting the visual list closes the nDCG gap.** nDCG@5 goes 0.495, 0.504, 0.511 for weights 1, 3 and 5. The 5x fusion is nominally best, but it leads ColQwen2.5 alone by at most 1.8 points of recall and 0.3 of nDCG, which is noise. Only weights 1, 3 and 5 were tried and the best was chosen on the same queries, so its lead is optimistic. Higher weights approach ColQwen2.5 alone.
- **At a larger K the retrievers converge in recall, not in ranking.** Recall@20 spans 0.549 to 0.647, against 0.400 to 0.484 at K = 5. Qwen3-VL (image) reaches 0.628 recall@20 but its nDCG@20 is 0.487 against 0.551 for ColQwen2.5, so it finds the pages but ranks them lower. ColQwen2.5 reaches about 57% of the possible recall at K = 5 and 64% at K = 20.
- **Fusion helps a little more at a larger K.** Equal-weight fusion has the best recall@20 (0.647 against 0.636 for ColQwen2.5), but its nDCG is lower at every K. The gaps are about one point.
- **Embedding time is not a clear difference here.** The ColQwen2.5 page-embedding cell took 12:27, against 11:46 and 12:26 for the two Qwen3-VL variants. The earlier ColSmol cell took 59:42, but the hardware was not recorded, so the runs are not comparable. Qwen3-VL still stores one 2048-number vector per page instead of many 128-number patch vectors.
- **Adding markdown gives Qwen3-VL a small gain:** +0.8 points recall@5 and +1.3 nDCG@5, within noise. It needs an OCR or PDF-to-text step for your own manuals.

Miss rate (share of queries with recall@5 = 0, lower is better), English only. Groups under about 30 queries move by only a few queries and are noise.

| group | ColQwen2.5 | fusion 5x | Qwen3-VL image | Qwen3-VL image + md | text dense | text hybrid |
|---|---|---|---|---|---|---|
| keyword (n=38) | 0.237 | 0.237 | 0.263 | 0.237 | **0.132** | 0.184 |
| question (n=161) | 0.199 | **0.186** | 0.261 | 0.248 | 0.304 | 0.236 |
| instruction (n=84) | **0.286** | 0.298 | 0.417 | 0.417 | 0.369 | 0.369 |
| open-ended (n=67) | 0.493 | 0.522 | **0.433** | 0.493 | 0.493 | 0.522 |
| enumerative (n=55) | 0.255 | **0.236** | 0.436 | 0.291 | 0.255 | 0.309 |
| compare-contrast (n=29) | **0.103** | 0.138 | 0.345 | 0.379 | 0.207 | 0.207 |
| extractive (n=125) | **0.120** | **0.120** | 0.264 | 0.184 | 0.232 | 0.176 |
| boolean (n=44) | 0.136 | **0.091** | 0.182 | 0.250 | 0.250 | 0.159 |
| multi-hop (n=28) | 0.179 | **0.143** | 0.250 | 0.214 | 0.250 | 0.179 |
| numerical (n=29) | **0.000** | 0.034 | 0.138 | 0.069 | 0.103 | 0.103 |
| content: image (n=21) | **0.286** | **0.286** | 0.333 | 0.429 | **0.286** | 0.381 |
| content: infographic (n=54) | 0.222 | **0.204** | 0.315 | 0.278 | 0.241 | 0.259 |
| content: table (n=88) | **0.250** | 0.295 | 0.295 | 0.273 | 0.307 | 0.273 |
| content: text (n=273) | 0.234 | **0.231** | 0.315 | 0.304 | 0.300 | 0.267 |

- **ColQwen2.5's gain is general, not specific to tables or figures.** On tables it misses 0.250 against 0.273 for text hybrid and 0.307 for text dense (2 to 5 queries). On infographics it misses 0.222 against 0.259 and 0.241 (1 to 2 queries). On plain text queries the gap is larger, 0.234 against 0.267 and 0.300. The visual retrievers still do not clearly beat text on tables and figures.
- **ColQwen2.5 beats text hybrid by 6 to 7 queries on instruction, extractive and question queries** and by 3 on compare-contrast and numerical queries.
- **Text is still better on keyword queries.** Text dense misses 0.132 against 0.237 for ColQwen2.5. Equal-weight fusion keeps most of that benefit (0.184), but weight 3 gives 0.211 and weight 5 gives 0.237, so the benefit is lost as the visual weight rises.
- **The 5x fusion differs from ColQwen2.5 alone by 0 to 4 queries per group.** It is worse on tables (0.295 against 0.250, 4 queries) and open-ended queries (2 queries), and better on boolean queries (2 queries).
- **Qwen3-VL (single vector) is weaker than ColQwen2.5 on compare-contrast queries,** 0.35 to 0.38 against 0.10. A single vector per page may struggle when the answer needs several parts of one page. With n=29 this is 7 to 8 queries.
- **Not measured:** how much the visual and text misses overlap, and a paired significance test between retrievers. Both are the next steps, together with trying `dense + ColQwen2.5` without BM25.

### All languages (earlier run, `rag.ipynb`)

5,244 pages, 1,698 queries with `qrels` labels (the 283 questions in 6 languages), `K = 5`, binary relevance. Kept for reference; the text-retriever scores are superseded by the English-only table above.

| retriever | recall@5 | nDCG@5 |
|---|---|---|
| sparse (BM25) | 0.156 | 0.146 |
| dense (Qwen3-Embedding-0.6B) | **0.311** | **0.310** |
| visual (ColSmol-500M) | 0.183 | 0.181 |

The English-only visual results are in the section above.

- **Dense was the best single retriever here,** about twice BM25 on both metrics. This reflects the language mix (see above) more than a real gap between keyword and semantic matching.
- **Visual beat BM25 but not dense in this run.** Page images with the 500M ColPali-style model gain some over keywords, but not enough to beat text embeddings, and they cost far more storage per page (many 128-dim patch vectors instead of one 1024-dim vector).
- **All scores are low in absolute terms.** Even the best retriever finds under a third of a query's relevant pages in the top 5. Part of this is the metric: queries often have many relevant pages (the example query has 10), and recall@5 can never exceed 5 / 10 = 0.5 for it. A larger K, or reading recall relative to `min(K, |relevant|)`, would show more.
- The hybrid (RRF) retriever scored 0.286 recall@5 and 0.259 nDCG@5 on this all-language set, below dense, because the weak BM25 list was given equal weight.

Other numbers from the run: the page-embedding cell took 1,311 batches (`batch_size=4`), and the visual model produced 128-dim vectors per patch for all 5,244 pages.

## Retrievers

| retriever | how it works | strengths | weaknesses |
|---|---|---|---|
| **Sparse, BM25s** | keyword matching with [`bm25s`](https://github.com/xhluca/bm25s). Text is lowercased, English stopwords are dropped and words are stemmed. Each (word, page) pair gets a weight from how often the word appears in the page, the page's length and how rare the word is across all pages. A page's score is the sum of the weights of the query's words. | exact terms: part numbers, manual codes | misses synonyms and paraphrases |
| **Dense, Qwen3-Embedding-0.6B** | the query and each page are embedded separately into 1024-number vectors. Pages are ranked by cosine similarity to the query. The query gets a task instruction in front of it, and pages are embedded as plain text. | matches meaning, not just words | can miss exact identifiers. Pages beyond 2048 tokens are truncated. Slower than BM25, especially without a GPU. |
| **Visual, ColQwen2.5** | embeds page **images** with [`vidore/colqwen2.5-v0.2`](https://huggingface.co/vidore/colqwen2.5-v0.2), a ColPali-style model built on Qwen2.5-VL (about 3B parameters). Each page becomes a bag of 128-dim patch vectors and each query a bag of token vectors. A page's score is the MaxSim: every query token takes its best-matching patch, and those maxima are summed. It replaced `colSmol-500M`. | sees layout, tables and figures that text extraction loses. The best single retriever in this benchmark. | much more storage per page than a single vector. Needs `colpali-engine` and a GPU to be practical. |
| **Visual, Qwen3-VL-Embedding-2B** (in `rag_image.ipynb`) | embeds each page **image** into one 2048-number vector with [`Qwen/Qwen3-VL-Embedding-2B`](https://huggingface.co/Qwen/Qwen3-VL-Embedding-2B) through `sentence-transformers`. Queries get a task instruction and are embedded as text. Pages are ranked by cosine similarity. A second variant also passes the page markdown alongside the image. | one vector per page, so a small index and fast search. | weaker than ColQwen2.5 and text dense on some query types (compare-contrast, instruction). A 2B model needs a GPU. |
| **Hybrid, RRF** (in `rag_text.ipynb` and `rag_image.ipynb`) | takes the top 50 pages from each retriever above and merges them with Reciprocal Rank Fusion. A page scores `weight / (60 + rank)` for each list it appears in, and the scores are added. `rag_image.ipynb` also fuses sparse + dense + ColQwen2.5 with the visual list weighted 1, 3 and 5. | uses ranks, so BM25, cosine and MaxSim scores don't need to be put on one scale | not guaranteed to beat the better single method. Here it does not clearly beat ColQwen2.5 alone. |

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

`transformers` needs to be version 4.51 or newer for Qwen3. `colpali-engine` pins `torch<2.14`, so if pip changes your torch version, restart the kernel before running the visual cells. The notebook also uninstalls `torchao` (Colab's old copy breaks the `transformers` import and nothing here uses it). The first run downloads the dataset (which includes page images), the embedding model (about 1.2 GB) and the ColQwen2.5 model. A GPU makes the page-embedding cell much faster. Run the cells from top to bottom. The benchmark cell prints a table with one row per retriever (sparse, dense, visual) and the columns `recall@5` and `nDCG@5`.

## Settings you may want to change

| setting | where | default |
|---|---|---|
| cutoff `K` | benchmark cell | 5 |
| embedding model `EMBED_MODEL` | dense retrieval cell | `Qwen/Qwen3-Embedding-0.6B` |
| query instruction `QUERY_PROMPT` | dense retrieval cell | "Given a question about a technical manual, retrieve the page that answers it" |
| max page length `max_seq_length` | dense retrieval cell | 2048 tokens |
| page-embedding batch size | dense retrieval cell | 4 |
| visual model `VISUAL_MODEL` | visual retrieval cell | `vidore/colqwen2.5-v0.2` (swap in `colSmol-500M` with `ColIdefics3` for a smaller, faster model) |
| `PAGE_BATCH`, `SCORE_BATCH` | visual retrieval cell | 8 and 64 |
| RRF constant `RRF_C` and pool `RRF_POOL` | hybrid cell in `rag_text.ipynb`, fusion cell in `rag_image.ipynb` | 60 and 50 |
| fusion weights (`weights=`) | fusion cell in `rag_image.ipynb` | 1, 3 and 5 for the visual list; 1 for sparse and dense |
| cutoffs `KS` | benchmark cell in `rag_image.ipynb` | 5, 10 and 20 (`K = 5` is used for the error analysis) |
