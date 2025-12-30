# API Contract: Backend-Frontend Integration

## Overview
This document defines the API contract between the Docusaurus frontend and the FastAPI RAG backend. It specifies the endpoints, request/response formats, authentication, and error handling mechanisms.

## Base URL
The base URL for the backend API is configured through environment variables and follows this pattern:
- Development: `http://localhost:8000`
- Staging: `https://staging-api.example.com`
- Production: `https://api.example.com`

## Endpoints

### POST /query
**Description**: Submit a user query to the RAG agent and receive a response with source attribution.

**Request**:
- Method: `POST`
- Endpoint: `/query`
- Content-Type: `application/json`
- Headers:
  - `Content-Type: application/json`
  - `Accept: application/json`

**Request Body**:
```json
{
  "query": "string (required) - The user's question or query text",
  "userId": "string (optional) - User identifier for tracking",
  "sessionId": "string (optional) - Session identifier for context",
  "context": {
    "additional_context": "string (optional) - Additional context for the query"
  }
}
```

**Request Validation**:
- `query` is required and must be between 1-1000 characters
- `userId` and `sessionId` must follow UUID format if provided
- Total request size must not exceed 2KB

**Response**:
- Success: `200 OK`
- Client Error: `400 Bad Request`
- Server Error: `500 Internal Server Error`

**Success Response Body** (200 OK):
```json
{
  "response": "string - The AI-generated response text",
  "sources": [
    {
      "title": "string - Title of the source document/chapter",
      "url": "string - URL to the source location",
      "section": "string - Specific section name",
      "confidence": "number - Confidence score for this source (0-1)"
    }
  ],
  "confidence": "number - Overall confidence score for the response (0-1)",
  "queryId": "string - Unique identifier for this query",
  "timestamp": "string - ISO 8601 timestamp of the response",
  "metadata": {
    "processing_time": "number - Time taken to process the query in milliseconds",
    "model_used": "string - Name of the AI model used"
  }
}
```

**Error Response Body** (400, 500):
```json
{
  "error": {
    "code": "string - Error code identifier",
    "message": "string - Human-readable error message",
    "details": "object - Additional error details if applicable"
  },
  "timestamp": "string - ISO 8601 timestamp of the error"
}
```

### GET /health
**Description**: Check the health status of the backend service.

**Request**:
- Method: `GET`
- Endpoint: `/health`
- Headers: None required

**Response**:
- Success: `200 OK`
- Server Error: `503 Service Unavailable`

**Success Response Body** (200 OK):
```json
{
  "status": "string - Health status ('healthy', 'degraded', 'unhealthy')",
  "timestamp": "string - ISO 8601 timestamp",
  "services": {
    "gemini": "boolean - Status of Google Gemini service connection",
    "qdrant": "boolean - Status of Qdrant vector database connection",
    "cohere": "boolean - Status of Cohere service connection"
  }
}
```

### GET /api/docs
**Description**: OpenAPI/Swagger documentation for the API.

**Request**:
- Method: `GET`
- Endpoint: `/api/docs`
- Headers: None required

**Response**:
- Success: `200 OK` with HTML documentation page

## Authentication
This API does not require authentication for basic query functionality. All requests are treated as anonymous.

## Rate Limiting
- Anonymous users: 100 requests per hour per IP address
- Rate limit headers will be included in responses when limits approach

## CORS Policy
The backend allows requests from:
- Development: `http://localhost:3000`, `http://localhost:3001`
- Production: Domain configured in environment variables

## Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `QUERY_TOO_LONG` | 400 | Query exceeds maximum length of 1000 characters |
| `QUERY_EMPTY` | 400 | Query field is empty or contains only whitespace |
| `INVALID_FORMAT` | 400 | Request body is not valid JSON or missing required fields |
| `BACKEND_UNAVAILABLE` | 503 | Backend services (Gemini, Qdrant, Cohere) are temporarily unavailable |
| `PROCESSING_ERROR` | 500 | An unexpected error occurred during query processing |
| `TIMEOUT_ERROR` | 408 | Request timed out while processing |

## Request/Response Examples

### Example Request:
```json
{
  "query": "What are the key concepts of RAG systems?",
  "userId": "user-12345",
  "sessionId": "session-67890"
}
```

### Example Response:
```json
{
  "response": "RAG (Retrieval-Augmented Generation) systems combine information retrieval with language model generation. They first retrieve relevant documents from a knowledge base, then use those documents as context for generating accurate, grounded responses.",
  "sources": [
    {
      "title": "Understanding RAG Systems",
      "url": "/docs/rag-systems",
      "section": "Introduction",
      "confidence": 0.95
    },
    {
      "title": "Implementation Patterns",
      "url": "/docs/implementation",
      "section": "RAG Architecture",
      "confidence": 0.87
    }
  ],
  "confidence": 0.92,
  "queryId": "query-abc123",
  "timestamp": "2025-12-29T10:30:00Z",
  "metadata": {
    "processing_time": 2450,
    "model_used": "gemini-pro"
  }
}
```

### Example Error Response:
```json
{
  "error": {
    "code": "QUERY_TOO_LONG",
    "message": "Query exceeds maximum length of 1000 characters",
    "details": {
      "max_length": 1000,
      "actual_length": 1250
    }
  },
  "timestamp": "2025-12-29T10:30:00Z"
}
```

## Versioning
- API Version: v1
- Version is not included in the URL but may be added in future releases
- Breaking changes will result in a new major version

## Performance Expectations
- 95% of queries should return within 10 seconds
- API response time should be under 500ms for successful requests
- Large or complex queries may take longer but should include appropriate status updates