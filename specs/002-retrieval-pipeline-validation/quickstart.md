# Quickstart: Retrieval Pipeline Validation

## Prerequisites

- Python 3.11+
- Cohere API key
- Qdrant Cloud connection details
- Existing Qdrant collection with embedded book content (from Spec 1)

## Setup

1. **Install Dependencies**
   ```bash
   pip install cohere qdrant-client python-dotenv
   ```

2. **Environment Configuration**
   Create a `.env` file with the following:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=book_chunks  # or your collection name from Spec 1
   ```

3. **Verify Environment**
   Ensure your environment variables are properly set:
   ```bash
   python -c "import os; print('COHERE_API_KEY' in os.environ)"
   ```

## Basic Usage

1. **Run Validation Script**
   ```bash
   cd backend/src/cli
   python retrieve.py
   ```

2. **Run with Specific Query**
   ```bash
   python retrieve.py --query "What does the book say about artificial intelligence?"
   ```

3. **Run Batch Validation**
   ```bash
   python retrieve.py --validate-all
   ```

## Configuration Options

- `--query`: Specify a query text for retrieval
- `--top-k`: Number of results to retrieve (default: 5)
- `--threshold`: Similarity score threshold (default: 0.5)
- `--collection`: Qdrant collection name (default: from environment)
- `--validate-all`: Run all predefined test queries
- `--verbose`: Enable detailed logging

## Expected Output

The script will output:
- Retrieved chunks with metadata (URL, chunk index, text)
- Similarity scores for each result
- Validation results indicating if retrieval was successful
- Performance metrics (execution time, etc.)

## Validation Process

1. Query text is embedded using the same Cohere model as ingestion
2. Similarity search is performed against Qdrant collection
3. Results are validated for relevance and metadata completeness
4. Performance metrics are collected and reported

## Troubleshooting

- **API Key Issues**: Verify your Cohere and Qdrant API keys are correct
- **Collection Not Found**: Ensure the Qdrant collection name matches the one from Spec 1
- **No Results**: Check that the collection contains embedded book content
- **Performance Issues**: Monitor API rate limits and Qdrant connection