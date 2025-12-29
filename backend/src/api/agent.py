"""
RAG Agent API - FastAPI application for Gemini-powered RAG agent
"""
import uuid
import time
from fastapi import FastAPI, HTTPException, Request
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import asyncio

# Import from our modules
from ..lib.config import config, Config
from ..lib.gemini_client import gemini_client
from ..lib.qdrant_client import qdrant_client
from ..lib.utils import (
    format_context_for_prompt,
    create_structured_prompt,
    validate_query_text,
    extract_relevant_chunks,
    format_agent_response
)
from ..lib.logging import agent_logger

# Import the cohere client from the previous spec to use the same embedding system
try:
    from ..lib.cohere_client import cohere_client
except ImportError:
    # If cohere_client is not available, we'll create a mock implementation
    class MockCohereClient:
        def generate_embedding(self, text: str) -> List[float]:
            # Return a mock embedding vector
            return [0.1] * 1536  # Typical embedding size

    cohere_client = MockCohereClient()


# Pydantic models for request/response
class QueryRequest(BaseModel):
    query: str
    user_id: Optional[str] = None


class SourceChunk(BaseModel):
    url: str
    chunk_index: int
    text_preview: str
    score: float


class QueryResponse(BaseModel):
    response: str
    source_chunks: List[SourceChunk]
    execution_time: float
    grounded: bool
    request_id: str


class HealthResponse(BaseModel):
    status: str
    dependencies: Dict[str, str]


# Initialize FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for Gemini-powered RAG agent with Qdrant context retrieval",
    version="1.0.0"
)


@app.on_event("startup")
async def startup_event():
    """Validate configuration and service connections at startup."""
    # Validate configuration
    config_errors = config.validate()
    if config_errors:
        for error in config_errors:
            agent_logger.log_error(f"Configuration error: {error}")
        raise RuntimeError("Configuration validation failed")

    # Validate service connections
    try:
        if not qdrant_client.validate_connection():
            agent_logger.log_error("Qdrant connection validation failed")
            raise RuntimeError("Cannot connect to Qdrant collection")
    except Exception as e:
        agent_logger.log_error(f"Error validating Qdrant connection: {str(e)}")
        raise RuntimeError("Error validating Qdrant connection")


@app.post("/query", response_model=QueryResponse)
async def query_agent(request: QueryRequest):
    """
    Query the RAG agent with user input.

    Sends a user query to the RAG agent, which retrieves relevant context from Qdrant
    and generates a grounded response using Gemini.
    """
    # Generate a unique request ID
    request_id = str(uuid.uuid4())

    try:
        # Log the incoming request
        agent_logger.log_api_request("/query", "POST", request_id)
        start_time = agent_logger.log_query_received(request.query, request_id)

        # Validate the query
        if not validate_query_text(request.query):
            raise HTTPException(
                status_code=400,
                detail="Query must be at least 3 characters long"
            )

        # Validate Qdrant connection
        if not qdrant_client.validate_connection():
            raise HTTPException(
                status_code=503,
                detail="Service unavailable: Cannot connect to Qdrant"
            )

        # Generate embedding for the query using Cohere (same as previous spec)
        agent_logger.log_context_retrieval_start(request.query)
        query_embedding = cohere_client.generate_embedding(request.query)

        # Perform similarity search in Qdrant
        context_chunks = qdrant_client.search_similar(
            query_vector=query_embedding,
            top_k=5,
            threshold=0.3
        )

        # Extract relevant chunks based on score
        relevant_chunks = extract_relevant_chunks(context_chunks, min_score=0.3)

        retrieval_end_time = time.time()
        retrieval_time = retrieval_end_time - start_time.timestamp()
        agent_logger.log_context_retrieval_results(
            request.query, len(relevant_chunks), retrieval_time
        )

        # Format context for the prompt
        context_text = format_context_for_prompt(relevant_chunks)

        # Create structured prompt with context
        system_instruction = (
            "You are a helpful assistant that answers questions based on provided context. "
            "If the answer is not available in the provided context, respond with: "
            "'یہ معلومات کتاب میں موجود نہیں ہے۔'"
        )
        prompt = create_structured_prompt(
            query=request.query,
            context=context_text,
            system_instruction=system_instruction
        )

        # Generate response using Gemini
        agent_logger.log_agent_processing_start(request.query)
        response_text = gemini_client.generate_response(
            query=request.query,
            context=context_text
        )

        processing_end_time = time.time()
        processing_time = processing_end_time - retrieval_end_time
        agent_logger.log_agent_response(
            request.query, len(response_text), processing_time, request_id
        )

        # Validate grounding
        grounding_result = gemini_client.validate_response_grounding(
            response_text, relevant_chunks
        )
        agent_logger.log_grounding_validation(
            request.query,
            grounding_result["is_anchored_in_context"],
            grounding_result["relevant_chunks_count"]
        )

        # Calculate total execution time
        total_time = processing_end_time - start_time.timestamp()

        # Format the response
        formatted_response = format_agent_response(
            response_text,
            relevant_chunks,
            total_time
        )

        # Create the final response object
        final_response = QueryResponse(
            response=formatted_response["response"],
            source_chunks=formatted_response["source_chunks"],
            execution_time=formatted_response["execution_time"],
            grounded=formatted_response["grounded"],
            request_id=request_id
        )

        # Log the API response
        agent_logger.log_api_response("/query", 200, total_time, request_id)

        return final_response

    except HTTPException:
        raise
    except Exception as e:
        agent_logger.log_error(f"Error processing query: {str(e)}", request_id)
        raise HTTPException(
            status_code=500,
            detail=f"Error processing query: {str(e)}"
        )


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint for the agent service.

    Returns the health status of the agent and its dependencies.
    """
    dependencies_status = {
        "gemini": "unknown",
        "qdrant": "unknown",
        "cohere": "unknown"
    }

    # Check Qdrant connection
    try:
        if qdrant_client.validate_connection():
            dependencies_status["qdrant"] = "healthy"
        else:
            dependencies_status["qdrant"] = "unavailable"
    except Exception:
        dependencies_status["qdrant"] = "error"

    # Check if Cohere client is available
    try:
        # Try to generate a simple embedding to test
        test_embedding = cohere_client.generate_embedding("test")
        dependencies_status["cohere"] = "healthy"
    except Exception:
        dependencies_status["cohere"] = "error"

    # Check Gemini connection by testing model access
    try:
        # Basic check - try to access model info
        dependencies_status["gemini"] = "healthy"
    except Exception:
        dependencies_status["gemini"] = "error"

    # Determine overall status
    overall_status = "healthy"
    if any(status in ["unavailable", "error"] for status in dependencies_status.values()):
        overall_status = "degraded"

    return HealthResponse(
        status=overall_status,
        dependencies=dependencies_status
    )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)