# Quickstart: RAG Agent API

## Prerequisites

- Python 3.11+
- Google Gemini API key
- Cohere API key
- Qdrant Cloud connection details
- Existing Qdrant collection with embedded book content (from Spec 1 & 2)

## Setup

1. **Install Dependencies**
   ```bash
   pip install google-generativeai fastapi uvicorn qdrant-client cohere python-dotenv
   ```

2. **Environment Configuration**
   Copy the existing `.env.example` file to `.env` and add the Gemini API key:
   ```bash
   cp backend/.env.example backend/.env
   ```

   Then add the Gemini API key to your `.env` file:
   ```env
   GEMINI_API_KEY=your_gemini_api_key_here
   # The following keys already exist in the example file:
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_HOST=your_qdrant_cloud_url
   QDRANT_COLLECTION_NAME=book_chunks  # or your collection name from previous specs
   ```

3. **Verify Environment**
   Ensure your environment variables are properly set:
   ```bash
   python -c "import os; print('OPENAI_API_KEY' in os.environ)"
   ```

## Basic Usage

1. **Start the API Server**
   ```bash
   cd backend/src/api
   uvicorn agent:app --host 0.0.0.0 --port 8000
   ```

2. **Query the Agent**
   ```bash
   curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d '{"query": "What does the book say about artificial intelligence?"}'
   ```

3. **Health Check**
   ```bash
   curl http://localhost:8000/health
   ```

## Configuration Options

- The agent uses the Google Generative AI (Gemini) with custom retrieval capabilities
- Context is retrieved from Qdrant using Cohere embeddings
- Responses are validated to ensure grounding in book content

## Expected Output

The API will return:
- Agent's response to the user query
- Source chunks that were used for context
- Execution time metrics
- Grounding verification

## Troubleshooting

- **API Key Issues**: Verify your Google Gemini API key is added and all keys (Gemini, Cohere, Qdrant) are correct
- **Collection Not Found**: Ensure the Qdrant collection name matches the one from previous specs
- **No Results**: Check that the collection contains embedded book content
- **Performance Issues**: Monitor API rate limits and Qdrant connection