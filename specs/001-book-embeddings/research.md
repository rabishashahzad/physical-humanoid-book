# Research: Book Embeddings and Vector Storage

**Feature**: Book Embeddings and Vector Storage
**Date**: 2025-12-27
**Researcher**: Claude

## Overview

Research for implementing an ingestion pipeline that crawls deployed book URLs, extracts content, generates embeddings using Cohere API, and stores them in Qdrant Cloud for RAG-based retrieval.

## Technology Research

### 1. Web Crawling and Content Extraction

**Decision**: Use `requests` and `beautifulsoup4` for web crawling and HTML parsing
**Rationale**: These are well-established, reliable libraries for web scraping in Python. `requests` handles HTTP operations reliably, and `beautifulsoup4` provides robust HTML parsing capabilities that can extract content while filtering out navigation and UI elements.
**Alternatives considered**:
- Selenium (overkill for static content, slower)
- Scrapy (more complex than needed for this use case)
- Playwright (also overkill for static content)

### 2. Text Processing and Chunking

**Decision**: Use custom text processing with `re` and `textwrap` modules for chunking
**Rationale**: For Docusaurus content, we need to preserve semantic meaning while chunking. Custom processing allows us to respect sentence boundaries and avoid breaking chunks mid-sentence. The approach will use character limits with intelligent split points.
**Alternatives considered**:
- RecursiveCharacterTextSplitter from langchain (introduces unnecessary dependency)
- Sentence transformers for chunking (not needed for basic text splitting)

### 3. Embedding Generation

**Decision**: Use Cohere's Python SDK for embedding generation
**Rationale**: The specification specifically calls for Cohere embedding models. The official Python SDK provides reliable access to the API with proper error handling and rate limiting support.
**Alternatives considered**:
- OpenAI embeddings (not specified in requirements)
- Hugging Face transformers (self-hosted, but more complex setup)
- Sentence transformers (local, but may not match Cohere quality)

### 4. Vector Storage

**Decision**: Use Qdrant Cloud with the official Python client
**Rationale**: The specification explicitly requires Qdrant Cloud for storage. The official Python client provides robust functionality for vector storage, indexing, and retrieval operations.
**Alternatives considered**:
- Pinecone (different service than specified)
- Weaviate (different service than specified)
- Chroma (local option, but not cloud-based as required)

### 5. Configuration Management

**Decision**: Use python-dotenv for environment variable management
**Rationale**: This is a standard, lightweight approach for managing configuration in Python applications. It allows for secure handling of API keys and connection strings without hardcoding them.
**Alternatives considered**:
- Built-in os.environ (less convenient for development)
- Pydantic settings (adds complexity without significant benefit)

### 6. Error Handling and Retries

**Decision**: Implement retry logic with exponential backoff for API calls
**Rationale**: API calls to Cohere and Qdrant may fail due to network issues or rate limits. Exponential backoff with jitter provides robust error handling while respecting rate limits.
**Alternatives considered**:
- Simple retry loops (less sophisticated)
- No retry logic (unreliable for production use)

## URL Discovery Strategy

**Decision**: Implement breadth-first crawling starting from the base URL
**Rationale**: This approach ensures we discover all publicly accessible pages systematically. We'll follow internal links while avoiding external domains and non-content routes.
**Implementation approach**:
1. Start with the base Vercel URL
2. Extract all internal links from each page
3. Filter for content pages (avoiding auth, admin, etc.)
4. Validate each URL before content extraction

## Text Extraction Strategy

**Decision**: Target main content areas in Docusaurus-generated HTML
**Rationale**: Docusaurus sites have predictable HTML structures with main content in specific elements. We'll target these elements while excluding navigation, headers, footers, and other non-content areas.
**Implementation approach**:
1. Identify common Docusaurus content selectors (main, article, .markdown)
2. Extract text content while preserving structure
3. Clean HTML entities and normalize whitespace
4. Remove navigation and UI elements

## Embedding Batch Processing

**Decision**: Process embeddings in batches to optimize API usage
**Rationale**: Cohere API supports batch operations which are more efficient than individual calls. This reduces the number of API requests and improves performance.
**Implementation approach**:
1. Group text chunks into batches (respecting API limits)
2. Process each batch with error handling
3. Track progress and resume capability for large datasets

## Testing Strategy

**Decision**: Implement comprehensive unit and integration tests
**Rationale**: The ingestion pipeline is critical for the RAG system. Comprehensive tests ensure reliability and correctness of each component.
**Implementation approach**:
1. Unit tests for each service module
2. Integration tests for the full pipeline
3. Mock external services for isolated testing
4. Performance tests for large datasets