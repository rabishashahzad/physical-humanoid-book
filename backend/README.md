# Book Embeddings Ingestion Pipeline and RAG Agent API

This project implements two main components:
1. An ingestion pipeline that crawls deployed book URLs from a Docusaurus site, extracts content, generates embeddings using Cohere API, and stores them in Qdrant Cloud for RAG-based retrieval.
2. A RAG Agent API that allows querying the stored embeddings and getting AI-generated responses based on the book content.

## Features

### Ingestion Pipeline
- **URL Discovery**: Crawls and discovers all publicly accessible book pages
- **Content Extraction**: Extracts clean text content from HTML pages, removing navigation and UI elements
- **Text Chunking**: Splits content into appropriately sized segments for embedding generation
- **Embedding Generation**: Generates vector embeddings using Cohere embedding models
- **Vector Storage**: Stores embeddings with metadata in Qdrant Cloud with proper indexing
- **Error Handling**: Handles API rate limits and retries failed requests appropriately

### RAG Agent API
- **Query Endpoint**: Accepts user queries and returns AI-generated responses
- **Context Retrieval**: Automatically retrieves relevant context from Qdrant before processing queries
- **Grounded Responses**: Ensures responses are based on retrieved context
- **Health Check**: Monitors service availability
- **Comprehensive Logging**: Tracks requests, responses, and performance metrics
- **Error Handling**: Robust error handling with retry logic for transient failures
- **Input Validation**: Validates query length and content

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

## RAG Agent API Usage

### Setup for RAG Agent API

The RAG Agent API requires additional environment variables beyond the ingestion pipeline. Update your `.env` file with:

```env
GEMINI_API_KEY=your_gemini_api_key
QDRANT_HOST=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_COLLECTION_NAME=book_embeddings  # Should match the collection used by the ingestion pipeline
AGENT_MODEL=gemini-pro  # Optional, defaults to 'gemini-pro'
```

### Running the RAG Agent API

Start the server:
```bash
cd backend
python -m src.api.agent
```

The server will start on `http://localhost:8000`.

### API Endpoints

#### POST /query

Query the RAG agent with user input.

**Request Body:**
```json
{
  "query": "Your question here",
  "user_id": "optional user identifier"
}
```

**Response:**
```json
{
  "response": "AI-generated response",
  "source_chunks": [
    {
      "url": "source URL",
      "chunk_index": 0,
      "text_preview": "Preview of the text chunk...",
      "score": 0.8
    }
  ],
  "execution_time": 0.5,
  "grounded": true,
  "request_id": "unique request identifier"
}
```

#### GET /health

Check the health status of the agent and its dependencies.

**Response:**
```json
{
  "status": "healthy",
  "dependencies": {
    "gemini": "healthy",
    "qdrant": "healthy",
    "cohere": "healthy"
  }
}
```

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
│   ├── api/              # API endpoints
│   │   └── agent.py      # RAG Agent API
│   └── lib/              # Configuration and utilities
│       ├── config.py     # Configuration management
│       ├── gemini_client.py  # Google Gemini client
│       ├── qdrant_client.py  # Qdrant client
│       ├── cohere_client.py  # Cohere client
│       ├── utils.py      # Utility functions
│       └── logging.py    # Logging utilities
├── tests/
│   ├── unit/             # Unit tests
│   │   └── test_api_agent.py  # API unit tests
│   └── integration/      # Integration tests
│       ├── test_vector_retrieval_integration.py
│       └── test_agent_lifecycle.py
└── requirements.txt      # Dependencies
```

## Performance Goals

- 95% URL crawl success rate
- 90% embedding generation success rate
- <5 seconds per chunk processing
- <2 hours for full book processing
- <100MB memory usage