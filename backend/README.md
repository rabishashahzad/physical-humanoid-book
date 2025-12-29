# Book Embeddings Ingestion Pipeline

This project implements an ingestion pipeline that crawls deployed book URLs from a Docusaurus site, extracts content, generates embeddings using Cohere API, and stores them in Qdrant Cloud for RAG-based retrieval.

## Features

- **URL Discovery**: Crawls and discovers all publicly accessible book pages
- **Content Extraction**: Extracts clean text content from HTML pages, removing navigation and UI elements
- **Text Chunking**: Splits content into appropriately sized segments for embedding generation
- **Embedding Generation**: Generates vector embeddings using Cohere embedding models
- **Vector Storage**: Stores embeddings with metadata in Qdrant Cloud with proper indexing
- **Error Handling**: Handles API rate limits and retries failed requests appropriately

## Prerequisites

- Python 3.11+
- pip package manager

## Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables

Create a `.env` file in the backend directory with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_cloud_api_key_here
QDRANT_HOST=your_qdrant_cloud_endpoint
BOOK_BASE_URL=https://your-book-vercel-url.com
QDRANT_COLLECTION_NAME=book_embeddings
CHUNK_SIZE=1000
CHUNK_OVERLAP=100
```

## Usage

### Basic Usage

```bash
cd backend
python -m src.cli.ingestion_pipeline
```

### With Custom Parameters

```bash
cd backend
python -m src.cli.ingestion_pipeline --base-url https://your-book.com --collection-name custom_collection
```

### Dry Run (for testing)

```bash
cd backend
python -m src.cli.ingestion_pipeline --dry-run
```

## Pipeline Steps

The ingestion pipeline performs the following steps:

1. **URL Discovery**: Crawls the base URL and discovers all accessible book pages
2. **Content Extraction**: Extracts clean text content from each page, removing navigation and UI elements
3. **Text Chunking**: Splits the content into appropriately sized chunks with overlap
4. **Embedding Generation**: Generates vector embeddings using Cohere API
5. **Vector Storage**: Stores embeddings in Qdrant Cloud with metadata

## Configuration Options

| Parameter | Environment Variable | Default | Description |
|-----------|---------------------|---------|-------------|
| `--base-url` | `BOOK_BASE_URL` | - | Base URL of the deployed book site |
| `--collection-name` | `QDRANT_COLLECTION_NAME` | `book_embeddings` | Name of the Qdrant collection |
| `--chunk-size` | `CHUNK_SIZE` | 1000 | Size of text chunks in characters |
| `--chunk-overlap` | `CHUNK_OVERLAP` | 100 | Overlap between chunks in characters |
| `--dry-run` | - | false | Run without storing embeddings |
| `--limit` | - | all | Limit number of pages to process |

## Monitoring Progress

The pipeline outputs progress information to the console:

```
[INFO] Starting URL discovery for: https://my-book.vercel.app
[INFO] Discovered 25 URLs to process
[INFO] Processing page 1/25: https://my-book.vercel.app/intro
[INFO] Extracted 1245 characters of content
[INFO] Generated 2 text chunks from page
[INFO] Generated embeddings for 2 chunks
[INFO] Stored 2 embeddings in Qdrant collection: book_embeddings
[SUCCESS] Pipeline completed successfully
```

## Troubleshooting

### Common Issues

1. **API Rate Limits**: If you encounter rate limit errors, the pipeline has built-in retry logic with exponential backoff. You may need to adjust your Cohere or Qdrant plan for higher limits.

2. **URL Access Issues**: If specific URLs are inaccessible, check that they are publicly available and not behind authentication.

3. **Memory Issues**: For very large books, the pipeline processes content in batches to minimize memory usage.

### Error Messages

- `COHERE_API_KEY not found`: Add your Cohere API key to the `.env` file
- `QDRANT connection failed`: Verify your Qdrant host and API key in the `.env` file
- `URL not accessible`: Check that the book URL is publicly accessible

## Testing

Run the unit tests:

```bash
cd backend
python -m pytest tests/unit/
```

Run the integration tests:

```bash
cd backend
python -m pytest tests/integration/
```

Run all tests:

```bash
cd backend
python -m pytest
```

## Architecture

The application follows a modular architecture:

```
backend/
├── src/
│   ├── models/           # Data models
│   │   └── embedding.py
│   ├── services/         # Business logic
│   │   ├── crawler.py
│   │   ├── text_processor.py
│   │   ├── embedding_generator.py
│   │   └── vector_storage.py
│   ├── cli/              # Command-line interface
│   │   └── ingestion_pipeline.py
│   └── lib/              # Configuration and utilities
│       ├── config.py
│       └── logging_config.py
├── tests/
│   ├── unit/             # Unit tests
│   └── integration/      # Integration tests
└── requirements.txt      # Dependencies
```

## Performance Goals

- 95% URL crawl success rate
- 90% embedding generation success rate
- <5 seconds per chunk processing
- <2 hours for full book processing
- <100MB memory usage