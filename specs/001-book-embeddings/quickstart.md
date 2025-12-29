# Quickstart: Book Embeddings Ingestion Pipeline

**Feature**: Book Embeddings and Vector Storage
**Date**: 2025-12-27

## Overview

This guide provides instructions for setting up and running the book embeddings ingestion pipeline that crawls deployed book URLs, extracts content, generates embeddings using Cohere API, and stores them in Qdrant Cloud.

## Prerequisites

- Python 3.11+
- pip package manager
- Git (for repository cloning)

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

If requirements.txt doesn't exist, install the required packages:

```bash
pip install requests beautifulsoup4 cohere qdrant-client python-dotenv pytest
```

### 4. Configure Environment Variables

Create a `.env` file in the project root with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_cloud_api_key_here
QDRANT_HOST=your_qdrant_cloud_endpoint
BOOK_BASE_URL=https://your-book-vercel-url.com
QDRANT_COLLECTION_NAME=book_embeddings
CHUNK_SIZE=1000
CHUNK_OVERLAP=100
```

**Note**: You can find your Qdrant Cloud endpoint in your Qdrant Cloud dashboard. The collection name will be created automatically if it doesn't exist.

## Running the Ingestion Pipeline

### 1. Basic Usage

```bash
cd backend
python -m src.cli.ingestion_pipeline
```

### 2. With Custom Parameters

```bash
cd backend
python -m src.cli.ingestion_pipeline --base-url https://your-book.com --collection-name custom_collection
```

### 3. Dry Run (for testing)

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

## Verification

To verify the pipeline completed successfully:

1. Check that the Qdrant collection contains the expected number of embeddings
2. Run a test query to ensure embeddings are properly stored:

```python
# Example verification code
from qdrant_client import QdrantClient
import os

client = QdrantClient(
    url=os.getenv("QDRANT_HOST"),
    api_key=os.getenv("QDRANT_API_KEY")
)

count = client.count(collection_name=os.getenv("QDRANT_COLLECTION_NAME"))
print(f"Total embeddings stored: {count.count}")
```

## Next Steps

Once the ingestion pipeline completes successfully:

1. The embeddings are ready for use in your RAG application
2. You can run similarity searches against the stored embeddings
3. The data is available for your chatbot or other retrieval applications