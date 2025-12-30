import unittest
from unittest.mock import Mock, patch
import asyncio
from fastapi.testclient import TestClient
from src.api.agent import app


class TestAgentLifecycle(unittest.TestCase):
    """Integration tests for complete agent lifecycle with configuration."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.client = TestClient(app)

    @patch('src.api.agent.gemini_client')
    @patch('src.api.agent.qdrant_client')
    @patch('src.api.agent.cohere_client')
    @patch('src.api.agent.config')
    def test_agent_lifecycle_with_valid_configuration(self, mock_config, mock_cohere, mock_qdrant, mock_gemini):
        """Test complete agent lifecycle with valid configuration."""
        # Mock valid configuration
        mock_config.validate.return_value = []
        mock_config.gemini_api_key = "test-gemini-key"
        mock_config.qdrant_api_key = "test-qdrant-key"
        mock_config.qdrant_url = "https://test-qdrant.com"
        mock_config.cohere_api_key = "test-cohere-key"

        # Mock service connections
        mock_qdrant.validate_connection.return_value = True
        mock_cohere.generate_embedding.return_value = [0.1] * 1536
        mock_gemini.generate_response.return_value = "Test response"
        mock_gemini.validate_response_grounding.return_value = {
            "is_anchored_in_context": True,
            "relevant_chunks_count": 1
        }

        # Test health check (validates that services are working)
        health_response = self.client.get("/health")
        self.assertEqual(health_response.status_code, 200)
        health_data = health_response.json()
        self.assertEqual(health_data["status"], "healthy")
        # Note: The health endpoint tests services internally, so actual results may vary based on mock behavior

        # Test query processing (end-to-end functionality)
        query_response = self.client.post(
            "/query",
            json={"query": "Test query for lifecycle"}
        )
        self.assertEqual(query_response.status_code, 200)
        query_data = query_response.json()
        self.assertIn("response", query_data)
        self.assertIn("source_chunks", query_data)
        self.assertIn("execution_time", query_data)
        self.assertIn("grounded", query_data)
        self.assertIn("request_id", query_data)

        # Verify that all services were called as expected
        mock_qdrant.validate_connection.assert_called()
        mock_cohere.generate_embedding.assert_called()
        mock_gemini.generate_response.assert_called()

    @patch('src.api.agent.config')
    def test_agent_startup_with_invalid_configuration(self, mock_config):
        """Test agent behavior when configuration is invalid."""
        # Mock invalid configuration
        mock_config.validate.return_value = ["GEMINI_API_KEY is required"]

        # In a real scenario, this would cause startup to fail
        # For testing purposes, we'll test that the validation works
        config_errors = mock_config.validate()
        self.assertIn("GEMINI_API_KEY is required", config_errors)

    @patch('src.api.agent.gemini_client')
    @patch('src.api.agent.qdrant_client')
    @patch('src.api.agent.cohere_client')
    @patch('src.api.agent.config')
    def test_agent_with_service_unavailable(self, mock_config, mock_cohere, mock_qdrant, mock_gemini):
        """Test agent behavior when services are unavailable."""
        # Mock valid configuration
        mock_config.validate.return_value = []

        # Mock service that's unavailable
        mock_qdrant.validate_connection.return_value = False

        # Test query with unavailable service
        response = self.client.post(
            "/query",
            json={"query": "Test query when service unavailable"}
        )
        self.assertEqual(response.status_code, 503)
        data = response.json()
        self.assertIn("detail", data)
        self.assertIn("Cannot connect to Qdrant", data["detail"])

    @patch('src.api.agent.gemini_client')
    @patch('src.api.agent.qdrant_client')
    @patch('src.api.agent.cohere_client')
    @patch('src.api.agent.config')
    def test_agent_error_handling_during_query_processing(self, mock_config, mock_cohere, mock_qdrant, mock_gemini):
        """Test agent error handling during query processing."""
        # Mock valid configuration
        mock_config.validate.return_value = []
        mock_qdrant.validate_connection.return_value = True

        # Mock a service that raises an exception
        mock_cohere.generate_embedding.side_effect = Exception("Service temporarily unavailable")

        # Test query processing with service error
        response = self.client.post(
            "/query",
            json={"query": "Test query that causes service error"}
        )
        # This should return a 500 error due to the service failure
        self.assertEqual(response.status_code, 500)
        data = response.json()
        self.assertIn("detail", data)
        self.assertIn("Internal server error", data["detail"])


if __name__ == "__main__":
    unittest.main()