import unittest
from unittest.mock import Mock, patch, MagicMock
from src.services.vector_storage import VectorStorage
from src.models.embedding import EmbeddingVector
from datetime import datetime


class TestVectorStorage(unittest.TestCase):
    """Unit tests for VectorStorage."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        with patch('src.services.vector_storage.QdrantClient'):
            self.vector_storage = VectorStorage()

    def test_store_embeddings_empty_input(self):
        """Test storing empty list of embeddings."""
        result = self.vector_storage.store_embeddings([])
        self.assertTrue(result)

    @patch('src.services.vector_storage.QdrantClient')
    def test_store_embeddings_success(self, mock_qdrant_client):
        """Test successful embedding storage."""
        # Create mock Qdrant client
        mock_client = Mock()
        mock_qdrant_client.return_value = mock_client

        # Create test embedding vectors
        embedding1 = EmbeddingVector(
            id="emb1",
            vector=[0.1, 0.2, 0.3],
            text_chunk_id="chunk1",
            model_name="test-model",
            created_at=datetime.now(),
            dimension=3
        )
        embedding2 = EmbeddingVector(
            id="emb2",
            vector=[0.4, 0.5, 0.6],
            text_chunk_id="chunk2",
            model_name="test-model",
            created_at=datetime.now(),
            dimension=3
        )

        embeddings = [embedding1, embedding2]

        with patch('src.services.vector_storage.QdrantClient') as mock_client_class:
            mock_instance = Mock()
            mock_client_class.return_value = mock_instance

            result = self.vector_storage.store_embeddings(embeddings)

        self.assertTrue(result)
        # Verify upload_points was called
        mock_instance.upload_points.assert_called_once()

    @patch('src.services.vector_storage.QdrantClient')
    def test_store_embeddings_error(self, mock_qdrant_client):
        """Test embedding storage when error occurs."""
        # Mock Qdrant client to raise an exception
        mock_instance = Mock()
        mock_instance.upload_points.side_effect = Exception("Upload failed")
        mock_qdrant_client.return_value = mock_instance

        embedding = EmbeddingVector(
            id="emb1",
            vector=[0.1, 0.2, 0.3],
            text_chunk_id="chunk1",
            model_name="test-model",
            created_at=datetime.now(),
            dimension=3
        )

        result = self.vector_storage.store_embeddings([embedding])

        self.assertFalse(result)

    @patch('src.services.vector_storage.QdrantClient')
    def test_retrieve_embedding_success(self, mock_qdrant_client):
        """Test successful embedding retrieval."""
        # Mock Qdrant client
        mock_instance = Mock()
        mock_record = Mock()
        mock_record.id = "emb1"
        mock_record.vector = [0.1, 0.2, 0.3]
        mock_record.payload = {
            "text_chunk_id": "chunk1",
            "model_name": "test-model",
            "created_at": "2023-01-01T00:00:00",
            "dimension": 3
        }
        mock_instance.retrieve.return_value = [mock_record]
        mock_qdrant_client.return_value = mock_instance

        result = self.vector_storage.retrieve_embedding("emb1")

        self.assertIsNotNone(result)
        self.assertEqual(result.id, "emb1")
        self.assertEqual(result.text_chunk_id, "chunk1")
        self.assertEqual(result.model_name, "test-model")

    @patch('src.services.vector_storage.QdrantClient')
    def test_retrieve_embedding_not_found(self, mock_qdrant_client):
        """Test embedding retrieval when embedding doesn't exist."""
        # Mock Qdrant client to return empty list
        mock_instance = Mock()
        mock_instance.retrieve.return_value = []
        mock_qdrant_client.return_value = mock_instance

        result = self.vector_storage.retrieve_embedding("nonexistent")

        self.assertIsNone(result)

    @patch('src.services.vector_storage.QdrantClient')
    def test_search_similar_success(self, mock_qdrant_client):
        """Test successful similar embedding search."""
        # Mock Qdrant client
        mock_instance = Mock()
        mock_result = Mock()
        mock_result.id = "emb1"
        mock_result.vector = [0.1, 0.2, 0.3]
        mock_result.payload = {
            "text_chunk_id": "chunk1",
            "model_name": "test-model",
            "created_at": "2023-01-01T00:00:00",
            "dimension": 3
        }
        mock_instance.search.return_value = [mock_result]
        mock_qdrant_client.return_value = mock_instance

        results = self.vector_storage.search_similar([0.1, 0.2, 0.3], limit=5)

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].id, "emb1")
        self.assertEqual(results[0].text_chunk_id, "chunk1")

    @patch('src.services.vector_storage.QdrantClient')
    def test_count_embeddings(self, mock_qdrant_client):
        """Test counting embeddings."""
        # Mock Qdrant client
        mock_instance = Mock()
        mock_count_result = Mock()
        mock_count_result.count = 42
        mock_instance.count.return_value = mock_count_result
        mock_qdrant_client.return_value = mock_instance

        count = self.vector_storage.count_embeddings()

        self.assertEqual(count, 42)

    @patch('src.services.vector_storage.QdrantClient')
    def test_count_embeddings_error(self, mock_qdrant_client):
        """Test counting embeddings when error occurs."""
        # Mock Qdrant client to raise an exception
        mock_instance = Mock()
        mock_instance.count.side_effect = Exception("Count failed")
        mock_qdrant_client.return_value = mock_instance

        count = self.vector_storage.count_embeddings()

        # Should return 0 on error
        self.assertEqual(count, 0)