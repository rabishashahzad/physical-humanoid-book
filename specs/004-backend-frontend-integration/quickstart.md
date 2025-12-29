# Quickstart Guide: Backend-Frontend Integration

## Overview
This guide provides step-by-step instructions to set up and run the integrated RAG system with Docusaurus frontend and FastAPI backend.

## Prerequisites
- Node.js 18+ (for Docusaurus frontend)
- Python 3.11+ (for FastAPI backend)
- Git
- Access to Google Gemini API key
- Access to Qdrant vector database
- Access to Cohere API key (optional, for enhanced embeddings)

## Environment Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup
Navigate to the backend directory and set up the Python environment:

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Backend Environment Variables
Create a `.env` file in the `backend/` directory with the following variables:

```env
GEMINI_API_KEY=your_google_gemini_api_key_here
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key_here  # Optional
QDRANT_COLLECTION_NAME=book_content
ENVIRONMENT=development
LOG_LEVEL=INFO
```

### 4. Frontend Setup
Navigate to the frontend directory and install dependencies:

```bash
cd book-frontend  # or wherever your Docusaurus frontend is located
npm install
# or if using yarn
yarn install
```

### 5. Frontend Environment Configuration
Configure the backend API URL in your Docusaurus configuration. In `docusaurus.config.js`, add or update the API configuration:

```javascript
module.exports = {
  // ... other config
  customFields: {
    backendApiUrl: process.env.BACKEND_API_URL || 'http://localhost:8000',
    // Additional API configuration can go here
  },
  // ... rest of config
};
```

## Running the Integrated System

### 1. Start the Backend
From the `backend/` directory:

```bash
cd src
uvicorn api.agent:app --reload --host 0.0.0.0 --port 8000
```

The backend will be available at `http://localhost:8000`.

### 2. Start the Frontend
From the `book-frontend/` directory:

```bash
npm run start
# or
yarn start
```

The frontend will be available at `http://localhost:3000`.

### 3. Verify the Connection
1. Open your browser to `http://localhost:3000`
2. Use the RAG query component to submit a test query
3. Verify that responses include source attribution from your book content
4. Check the browser's developer console for any API errors

## Configuration Options

### Environment Variables
- `BACKEND_API_URL`: URL of the FastAPI backend service
- `REACT_APP_BACKEND_API_URL`: React-specific environment variable for backend URL
- `NEXT_PUBLIC_BACKEND_API_URL`: Next.js-specific environment variable for backend URL

### API Timeout Configuration
Adjust the timeout for API requests in the frontend API service:

```javascript
const API_CONFIG = {
  timeout: 30000, // 30 seconds
  retries: 3,
  baseUrl: process.env.BACKEND_API_URL || 'http://localhost:8000'
};
```

## Testing the Integration

### 1. Health Check
Verify the backend is running:
```bash
curl http://localhost:8000/health
```

### 2. Test Query
Send a test query to verify the full integration:
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is this book about?", "userId": "test-user", "sessionId": "test-session"}'
```

### 3. Frontend Integration Test
In your frontend application:
1. Navigate to a page with the RAG query component
2. Submit a query related to your book content
3. Verify that the response appears with proper formatting
4. Confirm that source links are clickable and lead to the correct sections

## Troubleshooting

### Common Issues

#### 1. CORS Errors
If you see CORS errors in the browser console:
- Verify that the backend is configured to allow requests from your frontend domain
- Check that the `BACKEND_API_URL` is correctly set

#### 2. API Connection Failures
If the frontend cannot connect to the backend:
- Ensure the backend is running on the expected port
- Verify the `BACKEND_API_URL` environment variable
- Check firewall settings if running on different machines

#### 3. Empty Responses
If queries return empty responses:
- Verify that your book content has been properly indexed in Qdrant
- Check the backend logs for any errors during retrieval
- Confirm that the Qdrant collection contains the expected data

### Debugging Steps
1. Check backend logs for error messages
2. Verify all API keys are correctly configured
3. Confirm network connectivity between frontend and backend
4. Test the backend API directly using curl or Postman
5. Check browser console for frontend errors

## Production Deployment

### Environment Configuration
For production deployments, ensure the following environment variables are set:

**Backend (.env):**
```env
ENVIRONMENT=production
GEMINI_API_KEY=your_production_gemini_api_key
QDRANT_URL=your_production_qdrant_url
QDRANT_API_KEY=your_production_qdrant_api_key
```

**Frontend:**
```bash
BACKEND_API_URL=https://your-production-backend-domain.com
```

### Deployment Steps
1. Build the frontend for production:
   ```bash
   npm run build
   # or
   yarn build
   ```

2. Deploy the backend to your preferred hosting platform
3. Configure your web server to serve the built frontend
4. Ensure CORS settings allow requests from your production domain

## Next Steps

1. **Customize the UI**: Modify the RAG query component to match your site's design
2. **Add Analytics**: Implement tracking for query usage and user engagement
3. **Optimize Performance**: Add caching and performance monitoring
4. **Enhance Security**: Implement authentication if needed for your use case
5. **Scale the System**: Configure load balancing and monitoring for production use