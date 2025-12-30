# Data Model: Backend-Frontend Integration

## Entities

### Query Request
**Description**: User input text submitted to the RAG backend, including optional user context

**Fields**:
- `query` (string, required): The user's text query/question
- `userId` (string, optional): Identifier for the user making the request
- `sessionId` (string, optional): Session identifier for tracking conversation context
- `timestamp` (datetime, required): Time when the query was submitted
- `context` (object, optional): Additional context information for the query

**Validation Rules**:
- `query` must be between 1 and 1000 characters
- `query` must not be empty or whitespace-only
- `userId` and `sessionId` must follow UUID format if provided
- `timestamp` must be in ISO 8601 format

### Response Object
**Description**: AI-generated response from the RAG agent with source attribution and metadata

**Fields**:
- `response` (string, required): The AI-generated response text
- `sources` (array, required): List of source references used in the response
- `confidence` (number, optional): Confidence score between 0 and 1
- `timestamp` (datetime, required): Time when the response was generated
- `queryId` (string, required): Identifier linking to the original query
- `metadata` (object, optional): Additional response metadata

**Validation Rules**:
- `response` must not be empty
- `sources` must be an array of objects with required fields
- `confidence` must be between 0 and 1 if provided
- `queryId` must match the format of the original query identifier

### API Configuration
**Description**: Environment-based settings for backend endpoint URLs and connection parameters

**Fields**:
- `backendUrl` (string, required): Base URL for the FastAPI backend service
- `timeout` (number, optional): Request timeout in milliseconds (default: 30000)
- `retries` (number, optional): Number of retry attempts for failed requests (default: 3)
- `environment` (string, required): Environment identifier (development, staging, production)

**Validation Rules**:
- `backendUrl` must be a valid URL with proper protocol (http/https)
- `timeout` must be a positive integer between 1000 and 60000
- `retries` must be a non-negative integer between 0 and 5
- `environment` must be one of: 'development', 'staging', 'production'

## Relationships

### Query Request → Response Object
- One-to-one relationship: Each query request generates one response object
- The `queryId` in the response object references the original query request
- The relationship is established through the API communication flow

### API Configuration → Query Request
- One-to-many relationship: One configuration applies to multiple query requests
- Configuration settings affect how query requests are sent to the backend
- The relationship is established at the application level

## State Transitions

### Query Request States
- `PENDING`: Query has been submitted but not yet sent to backend
- `SENT`: Query has been sent to the backend service
- `PROCESSING`: Backend is processing the query
- `COMPLETED`: Response has been received from backend
- `FAILED`: Query processing failed (network error, backend error, etc.)

### Response Object States
- `DRAFT`: Response object created but not yet populated with content
- `POPULATED`: Response content has been filled from backend
- `VALIDATED`: Response has been validated for display
- `DISPLAYED`: Response has been shown to the user

## Business Rules

1. **Query Validation**: All queries must be validated before being sent to the backend
2. **Response Integrity**: Responses must maintain source attribution to maintain trust
3. **Configuration Consistency**: API configuration must remain consistent during a user session
4. **Error Handling**: Failed queries must result in appropriate error responses to the user
5. **Timeout Management**: Queries must respect configured timeout limits
6. **Retry Logic**: Failed requests should follow configured retry policies

## Constraints

- Query size: Maximum 1000 characters to prevent abuse
- Response caching: Responses may be cached for performance optimization
- Rate limiting: Query frequency may be limited to prevent abuse
- Data privacy: User-specific data should be handled according to privacy policies