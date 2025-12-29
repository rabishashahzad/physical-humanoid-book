# Data Model: Retrieval Pipeline Validation

## Entities

### Query
- **Description**: Text input provided for retrieval testing, containing the search intent
- **Fields**:
  - `text` (string): The query text to be embedded and searched
  - `expected_topic` (string, optional): The expected topic or subject area for validation
  - `validation_criteria` (object): Criteria for validating retrieval results
- **Validation rules**: Text must be non-empty and contain at least 3 characters

### Embedding
- **Description**: Vector representation of query text in the same space as stored book chunks
- **Fields**:
  - `vector` (array[float]): The numerical vector representation of the text
  - `model` (string): The Cohere model used for embedding generation
  - `text_hash` (string): Hash of the original text for verification
- **Validation rules**: Vector must match the dimensionality of the Qdrant collection

### BookChunk (from existing collection)
- **Description**: Segmented portion of book content stored in Qdrant with associated metadata
- **Fields**:
  - `url` (string): Source URL of the book content
  - `chunk_index` (integer): Sequential index of the chunk in the source
  - `text` (string): The actual text content of the chunk
  - `vector` (array[float]): The embedded vector representation
- **Validation rules**: All metadata fields must be present and non-empty

### RetrievalResult
- **Description**: Set of book chunks returned by similarity search with relevance scores and metadata
- **Fields**:
  - `chunks` (array[BookChunk]): Array of retrieved book chunks
  - `scores` (array[float]): Similarity scores corresponding to each chunk
  - `query_text` (string): The original query text
  - `execution_time` (float): Time taken for the retrieval operation
  - `relevance_accuracy` (float, optional): Calculated relevance score for validation
- **Validation rules**: Must contain at least one chunk with a valid score

### ValidationResult
- **Description**: Outcome of the validation process for a specific query
- **Fields**:
  - `query` (Query): The original query used for retrieval
  - `result` (RetrievalResult): The retrieval results
  - `is_valid` (boolean): Whether the results meet validation criteria
  - `validation_details` (object): Detailed validation results
  - `performance_metrics` (object): Performance metrics for the operation
- **Validation rules**: Must include both validity assessment and performance metrics

## Relationships

- A `Query` generates one `Embedding` for retrieval
- An `Embedding` is used to retrieve multiple `BookChunk` objects from Qdrant
- Multiple `BookChunk` results form one `RetrievalResult`
- One `RetrievalResult` is evaluated to produce one `ValidationResult`

## State Transitions

- `Query` → `Embedding` (when query text is converted to vector)
- `Embedding` → `RetrievalResult` (when similarity search is performed)
- `RetrievalResult` → `ValidationResult` (when results are validated against criteria)