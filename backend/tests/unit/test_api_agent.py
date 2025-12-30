import unittest
import asyncio
from unittest.mock import Mock, patch, MagicMock
from fastapi.testclient import TestClient
from src.api.agent import app, QueryRequest


class TestAgentAPI(unittest.TestCase):
    """Unit tests for the RAG Agent API."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.client = TestClient(app)

    @patch('src.api.agent.gemini_client')
    @patch('src.api.agent.qdrant_client')
    @patch('src.api.agent.cohere_client')
    @patch('src.api.agent.config')
    def test_health_endpoint(self, mock_config, mock_cohere, mock_qdrant, mock_gemini):
        """Test the health check endpoint."""
        # Mock configuration validation
        mock_config.validate.return_value = []

        # Mock service connections
        mock_qdrant.validate_connection.return_value = True
        mock_cohere.generate_embedding.return_value = [0.1] * 1536

        # Mock Gemini connection check
        mock_gemini.generate_response.return_value = "Test response"

        response = self.client.get("/health")

        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn("status", data)
        self.assertIn("dependencies", data)
        self.assertEqual(data["status"], "healthy")

    @patch('src.api.agent.gemini_client')
    @patch('src.api.agent.qdrant_client')
    @patch('src.api.agent.cohere_client')
    @patch('src.api.agent.config')
    @patch('src.api.agent.extract_relevant_chunks')
    @patch('src.api.agent.format_agent_response')
    def test_query_endpoint_success(self, mock_format_response, mock_extract_chunks,
                                   mock_config, mock_cohere, mock_qdrant, mock_gemini):
        """Test the query endpoint with a successful response."""
        # Mock configuration validation
        mock_config.validate.return_value = []

        # Mock service connections
        mock_qdrant.validate_connection.return_value = True
        mock_cohere.generate_embedding.return_value = [0.1] * 1536

        # Mock Qdrant search results
        mock_qdrant.search_similar.return_value = [
            {
                "url": "https://example.com/page1",
                "chunk_index": 0,
                "text": "This is relevant context for the query.",
                "score": 0.8
            }
        ]

        # Mock chunk extraction
        mock_extract_chunks.return_value = [
            {
                "url": "https://example.com/page1",
                "chunk_index": 0,
                "text": "This is relevant context for the query.",
                "score": 0.8
            }
        ]

        # Mock Gemini response
        mock_gemini.generate_response.return_value = "This is the AI response based on the context."
        mock_gemini.validate_response_grounding.return_value = {
            "is_anchored_in_context": True,
            "relevant_chunks_count": 1
        }

        # Mock response formatting
        mock_format_response.return_value = {
            "response": "This is the AI response based on the context.",
            "source_chunks": [
                {
                    "url": "https://example.com/page1",
                    "chunk_index": 0,
                    "text_preview": "This is relevant context for the query....",
                    "score": 0.8
                }
            ],
            "execution_time": 0.5,
            "grounded": True
        }

        # Send test request
        response = self.client.post(
            "/query",
            json={"query": "What is the meaning of life?"}
        )

        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn("response", data)
        self.assertIn("source_chunks", data)
        self.assertIn("execution_time", data)
        self.assertTrue(data["grounded"])
        self.assertIn("request_id", data)

    @patch('src.api.agent.config')
    def test_query_endpoint_short_query(self, mock_config):
        """Test the query endpoint with a query that's too short."""
        # Mock configuration validation
        mock_config.validate.return_value = []

        response = self.client.post(
            "/query",
            json={"query": "hi"}  # Too short (less than 3 characters)
        )

        self.assertEqual(response.status_code, 400)
        data = response.json()
        self.assertIn("detail", data)
        self.assertIn("Query must be at least 3 characters long", data["detail"])

    @patch('src.api.agent.config')
    def test_query_endpoint_missing_query(self, mock_config):
        """Test the query endpoint with missing query parameter."""
        # Mock configuration validation
        mock_config.validate.return_value = []

        response = self.client.post(
            "/query",
            json={"user_id": "test_user"}  # Missing query
        )

        # Should fail validation since query is required
        self.assertEqual(response.status_code, 422)  # Validation error

    @patch('src.api.agent.config')
    def test_query_endpoint_long_query(self, mock_config):
        """Test the query endpoint with a query that's too long."""
        # Mock configuration validation
        mock_config.validate.return_value = []

        # Create a query that exceeds 1000 characters
        long_query = "A" * 1001

        response = self.client.post(
            "/query",
            json={"query": long_query}
        )

        self.assertEqual(response.status_code, 400)
        data = response.json()
        self.assertIn("detail", data)
        self.assertIn("Query is too long", data["detail"])

    def test_query_endpoint_invalid_json(self):
        """Test the query endpoint with invalid JSON."""
        response = self.client.post(
            "/query",
            content="invalid json"
        )

        self.assertEqual(response.status_code, 422)  # Validation error


if __name__ == "__main__":
    unittest.main()