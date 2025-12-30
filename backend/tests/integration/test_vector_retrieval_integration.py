import unittest
from unittest.mock import Mock, patch
from fastapi.testclient import TestClient
from src.api.agent import app


class TestVectorRetrievalIntegration(unittest.TestCase):
    """Integration tests for vector retrieval functionality in the RAG Agent."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.client = TestClient(app)

    @patch('src.api.agent.gemini_client')
    @patch('src.api.agent.qdrant_client')
    @patch('src.api.agent.cohere_client')
    @patch('src.api.agent.config')
    @patch('src.api.agent.extract_relevant_chunks')
    @patch('src.api.agent.format_agent_response')
    def test_vector_retrieval_integration_success(self, mock_format_response, mock_extract_chunks,
                                                 mock_config, mock_cohere, mock_qdrant, mock_gemini):
        """Test that vector retrieval is properly integrated with query processing."""
        # Mock configuration validation
        mock_config.validate.return_value = []

        # Mock service connections
        mock_qdrant.validate_connection.return_value = True
        mock_cohere.generate_embedding.return_value = [0.1, 0.2, 0.3] * 512  # 1536-dimensional vector

        # Mock Qdrant search results with proper structure
        mock_qdrant.search_similar.return_value = [
            {
                "url": "https://example.com/doc1",
                "chunk_index": 0,
                "text": "This is relevant information about Python programming.",
                "score": 0.85,
                "vector": [0.1, 0.2, 0.3] * 512
            },
            {
                "url": "https://example.com/doc2",
                "chunk_index": 1,
                "text": "Python is a high-level programming language.",
                "score": 0.72,
                "vector": [0.4, 0.5, 0.6] * 512
            }
        ]

        # Mock chunk extraction
        mock_extract_chunks.return_value = [
            {
                "url": "https://example.com/doc1",
                "chunk_index": 0,
                "text": "This is relevant information about Python programming.",
                "score": 0.85
            },
            {
                "url": "https://example.com/doc2",
                "chunk_index": 1,
                "text": "Python is a high-level programming language.",
                "score": 0.72
            }
        ]

        # Mock Gemini response
        mock_gemini.generate_response.return_value = "Python is a high-level programming language."
        mock_gemini.validate_response_grounding.return_value = {
            "is_anchored_in_context": True,
            "relevant_chunks_count": 2
        }

        # Mock response formatting
        mock_format_response.return_value = {
            "response": "Python is a high-level programming language.",
            "source_chunks": [
                {
                    "url": "https://example.com/doc1",
                    "chunk_index": 0,
                    "text_preview": "This is relevant information about Python programming....",
                    "score": 0.85
                },
                {
                    "url": "https://example.com/doc2",
                    "chunk_index": 1,
                    "text_preview": "Python is a high-level programming language....",
                    "score": 0.72
                }
            ],
            "execution_time": 0.8,
            "grounded": True
        }

        # Send test query
        response = self.client.post(
            "/query",
            json={"query": "What is Python programming?"}
        )

        # Verify response
        self.assertEqual(response.status_code, 200)
        data = response.json()

        # Verify response structure
        self.assertIn("response", data)
        self.assertIn("source_chunks", data)
        self.assertIn("execution_time", data)
        self.assertIn("grounded", data)
        self.assertIn("request_id", data)

        # Verify that source chunks contain proper information
        self.assertEqual(len(data["source_chunks"]), 2)
        for chunk in data["source_chunks"]:
            self.assertIn("url", chunk)
            self.assertIn("chunk_index", chunk)
            self.assertIn("text_preview", chunk)
            self.assertIn("score", chunk)

        # Verify that the response is grounded in context
        self.assertTrue(data["grounded"])

        # Verify that Qdrant search was called with the correct parameters
        mock_qdrant.search_similar.assert_called_once()
        args, kwargs = mock_qdrant.search_similar.call_args
        self.assertIn('query_vector', kwargs)
        self.assertIn('top_k', kwargs)
        self.assertIn('threshold', kwargs)

        # Verify Cohere embedding was generated for the query
        mock_cohere.generate_embedding.assert_called_once_with("What is Python programming?")

    @patch('src.api.agent.gemini_client')
    @patch('src.api.agent.qdrant_client')
    @patch('src.api.agent.cohere_client')
    @patch('src.api.agent.config')
    @patch('src.api.agent.extract_relevant_chunks')
    @patch('src.api.agent.format_agent_response')
    def test_vector_retrieval_no_relevant_matches(self, mock_format_response, mock_extract_chunks,
                                                 mock_config, mock_cohere, mock_qdrant, mock_gemini):
        """Test behavior when no relevant matches are found in vector retrieval."""
        # Mock configuration validation
        mock_config.validate.return_value = []

        # Mock service connections
        mock_qdrant.validate_connection.return_value = True
        mock_cohere.generate_embedding.return_value = [0.1, 0.2, 0.3] * 512

        # Mock Qdrant search results with low scores (below threshold)
        mock_qdrant.search_similar.return_value = [
            {
                "url": "https://example.com/doc1",
                "chunk_index": 0,
                "text": "This is somewhat related information.",
                "score": 0.1,  # Below threshold of 0.3
                "vector": [0.1, 0.2, 0.3] * 512
            }
        ]

        # Mock chunk extraction returning empty list (no relevant chunks)
        mock_extract_chunks.return_value = []

        # Mock Gemini response without context
        mock_gemini.generate_response.return_value = "I don't have specific information about that in the provided context."
        mock_gemini.validate_response_grounding.return_value = {
            "is_anchored_in_context": False,
            "relevant_chunks_count": 0
        }

        # Mock response formatting
        mock_format_response.return_value = {
            "response": "I don't have specific information about that in the provided context.",
            "source_chunks": [],
            "execution_time": 0.5,
            "grounded": False
        }

        # Send test query
        response = self.client.post(
            "/query",
            json={"query": "What is a very specific topic not in the docs?"}
        )

        # Verify response
        self.assertEqual(response.status_code, 200)
        data = response.json()

        # Verify response structure
        self.assertIn("response", data)
        self.assertIn("source_chunks", data)
        self.assertIn("execution_time", data)
        self.assertIn("grounded", data)
        self.assertIn("request_id", data)

        # Verify that no source chunks were returned
        self.assertEqual(len(data["source_chunks"]), 0)

        # Verify that the response is not grounded in context
        self.assertFalse(data["grounded"])

    @patch('src.api.agent.gemini_client')
    @patch('src.api.agent.qdrant_client')
    @patch('src.api.agent.cohere_client')
    @patch('src.api.agent.config')
    def test_vector_retrieval_error_handling(self, mock_config, mock_cohere, mock_qdrant, mock_gemini):
        """Test error handling when vector retrieval fails."""
        # Mock configuration validation
        mock_config.validate.return_value = []

        # Mock Qdrant connection validation to fail
        mock_qdrant.validate_connection.return_value = False

        # Send test query
        response = self.client.post(
            "/query",
            json={"query": "What happens when retrieval fails?"}
        )

        # Should return 503 Service Unavailable
        self.assertEqual(response.status_code, 503)
        data = response.json()
        self.assertIn("detail", data)
        self.assertIn("Cannot connect to Qdrant", data["detail"])


if __name__ == "__main__":
    unittest.main()