





# Data Model: RAG Agent API

## Entities

### UserQuery
- **Description**: Input text from the user that requires a response based on book content
- **Fields**:
  - `query_text` (string): The user's query text
  - `user_id` (string, optional): Identifier for the user (if needed for tracking)
  - `timestamp` (datetime): When the query was received
- **Validation rules**: Query text must be non-empty and contain at least 3 characters

### RetrievedContext
- **Description**: Relevant book content chunks retrieved from Qdrant based on semantic similarity to the query
- **Fields**:
  - `chunks` (array[BookChunk]): Array of relevant book content chunks
  - `scores` (array[float]): Similarity scores for each chunk
  - `query_embedding` (array[float]): The embedding vector of the original query
  - `retrieval_timestamp` (datetime): When the context was retrieved
- **Validation rules**: Must contain at least one chunk with a valid score

### BookChunk (from existing collection)
- **Description**: Segmented portion of book content stored in Qdrant with associated metadata
- **Fields**:
  - `url` (string): Source URL of the book content
  - `chunk_index` (integer): Sequential index of the chunk in the source
  - `text` (string): The actual text content of the chunk
  - `vector` (array[float]): The embedded vector representation
- **Validation rules**: All metadata fields must be present and non-empty

### AgentResponse
- **Description**: Generated answer that incorporates the retrieved context and addresses the user's query
- **Fields**:
  - `response_text` (string): The agent's response to the user query
  - `source_chunks` (array[BookChunk]): Chunks that were used as context
  - `confidence_score` (float): Confidence in the response quality
  - `generation_timestamp` (datetime): When the response was generated
  - `grounding_verification` (object): Verification that response is grounded in book content
- **Validation rules**: Response must be non-empty and contain references to source chunks

### APIRequest
- **Description**: HTTP request containing the user query sent to the FastAPI endpoint
- **Fields**:
  - `query` (string): The user's query text
  - `user_metadata` (object, optional): Additional metadata about the request
  - `request_id` (string): Unique identifier for the request
- **Validation rules**: Query field must be present and non-empty

### APIResponse
- **Description**: HTTP response containing the agent's response to the user query
- **Fields**:
  - `response` (string): The agent's response text
  - `status` (string): Status of the request (success, error)
  - `request_id` (string): Correlation ID from the request
  - `execution_time` (float): Time taken to process the request
  - `source_chunks` (array[object]): Metadata about the chunks used for context
- **Validation rules**: Response field must be present for successful requests

### AgentConfiguration
- **Description**: Settings and API keys needed to connect to Google Gemini, Qdrant, and Cohere services
- **Fields**:
  - `gemini_api_key` (string): API key for Google Gemini services
  - `qdrant_url` (string): URL for Qdrant service
  - `qdrant_api_key` (string): API key for Qdrant service
  - `cohere_api_key` (string): API key for Cohere service
  - `qdrant_collection_name` (string): Name of the Qdrant collection
  - `agent_model` (string): Model identifier for the Gemini agent
- **Validation rules**: All API key fields must be present and non-empty

## Relationships

- An `APIRequest` contains one `UserQuery`
- A `UserQuery` generates one `RetrievedContext` through the retrieval process
- `RetrievedContext` contains multiple `BookChunk` objects from Qdrant
- `RetrievedContext` and `UserQuery` are used to generate one `AgentResponse`
- An `APIResponse` contains one `AgentResponse`

## State Transitions

- `APIRequest` → `UserQuery` (when request is parsed and validated)
- `UserQuery` → `RetrievedContext` (when context is retrieved from Qdrant)
- `UserQuery` + `RetrievedContext` → `AgentResponse` (when agent generates response)
- `AgentResponse` → `APIResponse` (when response is formatted for API)