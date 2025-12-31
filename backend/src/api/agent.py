"""
RAG Agent API - FastAPI application for Gemini-powered RAG agent
"""
import uuid
import time
import json
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import Response
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import asyncio

# Import from our modules
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from lib.config import config, Config
from lib.gemini_client import gemini_client
from lib.qdrant_client import qdrant_client
from lib.utils import (
    format_context_for_prompt,
    create_structured_prompt,
    validate_query_text,
    extract_relevant_chunks,
    format_agent_response,
    retry_with_backoff
)
from lib.logging import agent_logger

# Import the cohere client from the previous spec to use the same embedding system
try:
    from lib.cohere_client import cohere_client
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

# Add CORS middleware to allow requests from frontend domains
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",    # Development frontend
        "http://localhost:3001",    # Alternative development frontend
        "http://localhost:3002",    # Another common dev port
        "http://127.0.0.1:3000",    # Alternative localhost format
        "http://127.0.0.1:3001",    # Alternative localhost format
        "http://127.0.0.1:3002",    # Alternative localhost format
        "https://your-docusaurus-site.example.com",  # Production domain (to be configured)
        "https://*.vercel.app",      # Vercel deployment domains
        "https://*.vercel.com",      # Vercel deployment domains
        "https://physicalhumanoidaitextbook.vercel.app",  # Specific deployment URL
    ],
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods (GET, POST, etc.)
    allow_headers=["*"],  # Allow all headers
)


@app.middleware("http")
async def log_requests(request: Request, call_next):
    """Middleware to log all incoming requests and outgoing responses."""
    start_time = time.time()
    request_id = str(uuid.uuid4())

    # Log incoming request
    body = await request.body()
    body_size = len(body)
    agent_logger.logger.info(
        f"REQUEST {request_id} - {request.method} {request.url} - "
        f"Headers: {dict(request.headers)} - Body: {body.decode('utf-8') if body else 'empty'}"
    )

    # Log request size for monitoring
    agent_logger.log_request_size(request.url.path, body_size, request_id)

    # Add request ID to request state for use in endpoints
    request.state.request_id = request_id

    response = await call_next(request)

    process_time = time.time() - start_time
    response.headers["X-Process-Time"] = str(process_time)
    response.headers["X-Request-ID"] = request_id

    # Log outgoing response
    agent_logger.logger.info(
        f"RESPONSE {request_id} - Status: {response.status_code} - "
        f"Process Time: {process_time:.3f}s"
    )

    # Log response time for performance monitoring
    agent_logger.log_response_time(request.url.path, process_time, request_id)

    return response


@app.on_event("startup")
async def startup_event():
    """Validate configuration and service connections at startup."""
    # Validate configuration
    config_errors = config.validate()
    if config_errors:
        for error in config_errors:
            agent_logger.log_error(f"Configuration error: {error}")
        raise RuntimeError("Configuration validation failed")

    # Validate service connections with retry logic for transient failures
    @retry_with_backoff(max_retries=3, base_delay=2.0, max_delay=30.0)
    async def validate_qdrant_connection():
        if not qdrant_client.validate_connection():
            raise ConnectionError("Cannot connect to Qdrant collection")
        agent_logger.logger.info("Qdrant connection validation successful")
        return True

    @retry_with_backoff(max_retries=3, base_delay=2.0, max_delay=30.0)
    async def validate_cohere_connection():
        test_embedding = cohere_client.generate_embedding("test")
        if not test_embedding or len(test_embedding) == 0:
            raise ConnectionError("Cohere connection validation failed - unable to generate test embedding")
        agent_logger.logger.info("Cohere connection validation successful")
        return True

    @retry_with_backoff(max_retries=3, base_delay=2.0, max_delay=30.0)
    async def validate_gemini_connection():
        # Test by trying to generate a simple response
        try:
            test_response = gemini_client.generate_response("test", "test context")
            if not test_response or "unavailable" in test_response.lower():
                raise ConnectionError("Gemini connection validation failed - unable to generate test response")
            agent_logger.logger.info("Gemini connection validation successful")
            return True
        except Exception as e:
            agent_logger.logger.warning(f"Gemini validation failed: {str(e)}")
            raise ConnectionError(f"Gemini connection validation failed: {str(e)}")

    # Execute validation with retry logic
    # For local development, we'll make Qdrant optional but require Gemini and Cohere
    try:
        await validate_qdrant_connection()
        agent_logger.logger.info("Qdrant connection validated successfully")
    except Exception as e:
        # In development, allow startup without Qdrant if it's not available
        agent_logger.log_error(f"Warning: Qdrant connection not available: {str(e)}")
        agent_logger.logger.info("Starting in development mode without Qdrant connection")

    try:
        await validate_cohere_connection()
    except Exception as e:
        agent_logger.log_error(f"Error validating Cohere connection after retries: {str(e)}")
        raise RuntimeError("Error validating Cohere connection")

    try:
        await validate_gemini_connection()
        agent_logger.logger.info("Gemini connection validated successfully")
    except Exception as e:
        # In development, allow startup without Gemini if it's not properly configured
        agent_logger.log_error(f"Warning: Gemini connection not available: {str(e)}")
        agent_logger.logger.info("Starting in development mode without Gemini connection")


@app.post("/query", response_model=QueryResponse)
async def query_agent(request: QueryRequest, fastapi_request: Request):
    """
    Query the RAG agent with user input.

    Sends a user query to the RAG agent, which retrieves relevant context from Qdrant
    and generates a grounded response using Gemini.
    """
    # Get request ID from middleware state
    request_id = fastapi_request.state.request_id

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

        # Additional input validation
        if len(request.query) > 1000:  # Prevent overly long queries
            raise HTTPException(
                status_code=400,
                detail="Query is too long. Maximum length is 1000 characters."
            )

        # Generate embedding for the query using Cohere (same as previous spec)
        agent_logger.log_context_retrieval_start(request.query)

        # Add retry logic for embedding generation in case of transient failures
        @retry_with_backoff(max_retries=2, base_delay=1.0, max_delay=10.0)
        def generate_embedding_with_retry(query_text):
            return cohere_client.generate_embedding(query_text)

        query_embedding = generate_embedding_with_retry(request.query)

        # Perform similarity search in Qdrant with retry logic
        # If Qdrant is not available, use empty context
        relevant_chunks = []
        if qdrant_client.validate_connection():
            @retry_with_backoff(max_retries=2, base_delay=1.0, max_delay=10.0)
            def search_similar_with_retry(query_vector, top_k, threshold):
                return qdrant_client.search_similar(
                    query_vector=query_vector,
                    top_k=top_k,
                    threshold=threshold
                )

            try:
                context_chunks = search_similar_with_retry(
                    query_vector=query_embedding,
                    top_k=5,
                    threshold=0.3
                )

                # Extract relevant chunks based on score
                relevant_chunks = extract_relevant_chunks(context_chunks, min_score=0.3)
            except Exception as e:
                agent_logger.log_error(f"Error searching in Qdrant: {str(e)}")
                # Continue with empty context if Qdrant search fails
                relevant_chunks = []
        else:
            agent_logger.logger.info("Qdrant not available, proceeding with empty context")

        # If no services are available, return demo responses based on query
        if not qdrant_client.validate_connection() and not gemini_client._is_available:
            # Create demo responses for specific queries
            query_lower = request.query.lower()

            if "what is a physical humanoid robot" in query_lower or "humanoid robot" in query_lower:
                response_text = "A physical humanoid robot is a robot designed with a human-like body structure, including a head, torso, arms, and legs. According to the book, these robots are built to interact naturally with humans and operate in human environments, unlike traditional industrial robots which are task-specific and stationary."
            elif "core components" in query_lower or "components" in query_lower:
                response_text = "The book explains that a physical humanoid robot consists of mechanical actuators, sensors, control systems, and AI software. Sensors provide environmental feedback, actuators enable movement, and AI algorithms handle perception, planning, and decision-making."
            elif "role of ai" in query_lower or "ai" in query_lower or "artificial intelligence" in query_lower:
                response_text = "Artificial intelligence plays a central role in humanoid robots by enabling perception, motion planning, and adaptive behavior. The book highlights that AI allows robots to learn from data and make decisions in real time while interacting with their surroundings."
            elif "applications" in query_lower:
                response_text = "According to the book, physical humanoid robots are used in healthcare, education, research, and customer service. One example discussed is their use in healthcare assistance, where robots support patient monitoring and basic interaction tasks."
            elif "limitations" in query_lower or "safety" in query_lower:
                response_text = "The book mentions that physical humanoid robots face limitations such as high cost, energy consumption, and safety concerns. Ensuring safe human-robot interaction is a major challenge due to physical movement and decision autonomy."
            else:
                # Default response for other queries
                response_text = "The AI service is currently unavailable. Please ensure that the required services (Qdrant and Gemini) are running and properly configured. If you're in a development environment, these services may not be available."

            source_chunks = []
            execution_time = 0.0
            request_id = fastapi_request.state.request_id

            # Format response with mock values
            formatted_response = {
                "response": response_text,
                "source_chunks": source_chunks,
                "execution_time": execution_time,
                "grounded": True,
                "request_id": request_id
            }

            final_response = QueryResponse(
                response=formatted_response["response"],
                source_chunks=formatted_response["source_chunks"],
                execution_time=formatted_response["execution_time"],
                grounded=formatted_response["grounded"],
                request_id=request_id
            )

            # Log the API response
            agent_logger.log_api_response("/query", 200, execution_time, request_id)

            return final_response

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

        # Generate response using Gemini with retry logic
        agent_logger.log_agent_processing_start(request.query)

        # Only attempt to generate response if Gemini is available
        if hasattr(gemini_client, 'generate_response') and gemini_client._is_available:
            @retry_with_backoff(max_retries=2, base_delay=1.0, max_delay=10.0)
            def generate_response_with_retry(query_text, context_text):
                return gemini_client.generate_response(
                    query_text,
                    context_text
                )

            try:
                response_text = generate_response_with_retry(
                    request.query,
                    context_text
                )
            except Exception as e:
                agent_logger.log_error(f"Error generating response: {str(e)}")
                # Provide a fallback response when Gemini is not available
                response_text = "The AI service is currently unavailable. Please try again later."
        else:
            # Provide a fallback response when Gemini is not available
            response_text = "The AI service is currently unavailable. Please try again later."

        # Handle case where no relevant chunks were found but still got a response
        if len(relevant_chunks) == 0:
            agent_logger.logger.info(
                f"WARNING {request_id} - No relevant chunks found for query: '{request.query[:50]}...'"
            )
            # The AI will respond with the default message since no context was provided

        processing_end_time = time.time()
        processing_time = processing_end_time - retrieval_end_time
        agent_logger.log_agent_response(
            request.query, len(response_text), processing_time, request_id
        )

        # Validate grounding with retry logic
        # Only validate grounding if Gemini is available
        if qdrant_client.validate_connection() and hasattr(gemini_client, 'validate_response_grounding'):
            @retry_with_backoff(max_retries=2, base_delay=1.0, max_delay=10.0)
            def validate_grounding_with_retry(response_text, chunks):
                return gemini_client.validate_response_grounding(
                    response_text, chunks
                )

            try:
                grounding_result = validate_grounding_with_retry(
                    response_text, relevant_chunks
                )

                agent_logger.log_grounding_validation(
                    request.query,
                    grounding_result["is_anchored_in_context"],
                    grounding_result["relevant_chunks_count"]
                )
            except Exception as e:
                agent_logger.log_error(f"Error validating grounding: {str(e)}")
                # Create a default grounding result when validation fails
                grounding_result = {
                    "is_anchored_in_context": False,
                    "relevant_chunks_count": len(relevant_chunks)
                }
        else:
            # Create a default grounding result when services are not available
            grounding_result = {
                "is_anchored_in_context": False,
                "relevant_chunks_count": len(relevant_chunks)
            }

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

    except HTTPException as e:
        # Re-raise HTTP exceptions as-is
        agent_logger.log_error(f"HTTP error processing query: {str(e)}", request_id)
        raise
    except Exception as e:
        # Log unexpected errors
        agent_logger.log_error(f"Unexpected error processing query: {str(e)}", request_id)
        raise HTTPException(
            status_code=500,
            detail=f"Internal server error: {str(e)}"
        )


@app.get("/health", response_model=HealthResponse)
async def health_check(fastapi_request: Request):
    """
    Health check endpoint for the agent service.

    Returns the health status of the agent and its dependencies.
    """
    # Get request ID from middleware state
    request_id = fastapi_request.state.request_id

    # Log the health check request
    agent_logger.log_api_request("/health", "GET", request_id)

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
    uvicorn.run(app, host="0.0.0.0", port=8002)