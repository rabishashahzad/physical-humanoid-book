import unittest
from unittest.mock import Mock, patch
from src.services.vector_storage import VectorStorage
from src.models.embedding import EmbeddingVector
from datetime import datetime


class TestVectorStorageIntegration(unittest.TestCase):
    """Integration tests for VectorStorage with embedding generation."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        with patch('src.services.vector_storage.QdrantClient'):
            self.vector_storage = VectorStorage()

    def test_embedding_storage_retrieval_integration(self):
        """Test integration between embedding storage and retrieval."""
        # Create test embedding vectors
        embedding1 = EmbeddingVector(
            id="emb-test-1",
            vector=[0.1, 0.2, 0.3, 0.4],
            text_chunk_id="chunk-test-1",
            model_name="test-model",
            created_at=datetime.now(),
            dimension=4
        )
        embedding2 = EmbeddingVector(
            id="emb-test-2",
            vector=[0.5, 0.6, 0.7, 0.8],
            text_chunk_id="chunk-test-2",
            model_name="test-model",
            created_at=datetime.now(),
            dimension=4
        )

        embeddings = [embedding1, embedding2]

        # Mock the Qdrant client operations
        with patch('src.services.vector_storage.QdrantClient') as mock_client_class:
            mock_instance = Mock()

            # Mock the upload_points method
            mock_instance.upload_points = Mock(return_value=None)

            # Mock the retrieve method for later retrieval test
            def mock_retrieve(collection_name, ids):
                if ids[0] == "emb-test-1":
                    mock_record = Mock()
                    mock_record.id = "emb-test-1"
                    mock_record.vector = [0.1, 0.2, 0.3, 0.4]
                    mock_record.payload = {
                        "text_chunk_id": "chunk-test-1",
                        "model_name": "test-model",
                        "created_at": embedding1.created_at.isoformat(),
                        "dimension": 4
                    }
                    return [mock_record]
                return []

            mock_instance.retrieve.side_effect = mock_retrieve
            mock_client_class.return_value = mock_instance

            # Test storage
            store_result = self.vector_storage.store_embeddings(embeddings)
            self.assertTrue(store_result)

            # Test retrieval
            retrieved_embedding = self.vector_storage.retrieve_embedding("emb-test-1")
            self.assertIsNotNone(retrieved_embedding)
            self.assertEqual(retrieved_embedding.id, "emb-test-1")
            self.assertEqual(retrieved_embedding.text_chunk_id, "chunk-test-1")
            self.assertEqual(retrieved_embedding.vector, [0.1, 0.2, 0.3, 0.4])

    def test_embedding_storage_search_integration(self):
        """Test integration between embedding storage and search."""
        # Create test embeddings
        embeddings = []
        for i in range(5):
            emb = EmbeddingVector(
                id=f"emb-search-{i}",
                vector=[0.1 + i*0.1, 0.2 + i*0.1, 0.3 + i*0.1, 0.4 + i*0.1],
                text_chunk_id=f"chunk-search-{i}",
                model_name="test-model",
                created_at=datetime.now(),
                dimension=4
            )
            embeddings.append(emb)

        # Mock the Qdrant client operations
        with patch('src.services.vector_storage.QdrantClient') as mock_client_class:
            mock_instance = Mock()

            # Mock the upload_points method
            mock_instance.upload_points = Mock(return_value=None)

            # Mock the search method
            def mock_search(collection_name, query_vector, limit):
                # Return mock search results
                results = []
                for i in range(min(limit, 3)):  # Return up to 3 results
                    mock_result = Mock()
                    mock_result.id = f"emb-search-{i}"
                    mock_result.vector = [0.1 + i*0.1, 0.2 + i*0.1, 0.3 + i*0.1, 0.4 + i*0.1]
                    mock_result.payload = {
                        "text_chunk_id": f"chunk-search-{i}",
                        "model_name": "test-model",
                        "created_at": datetime.now().isoformat(),
                        "dimension": 4
                    }
                    results.append(mock_result)
                return results

            mock_instance.search.side_effect = mock_search
            mock_client_class.return_value = mock_instance

            # Test storage
            store_result = self.vector_storage.store_embeddings(embeddings)
            self.assertTrue(store_result)

            # Test search
            search_results = self.vector_storage.search_similar([0.1, 0.2, 0.3, 0.4], limit=5)
            self.assertEqual(len(search_results), 3)  # Based on our mock implementation
            for result in search_results:
                self.assertTrue(result.id.startswith("emb-search-"))

    def test_full_pipeline_integration(self):
        """Test integration of the full pipeline: storage, retrieval, and search."""
        # Create test embeddings
        test_embeddings = []
        for i in range(3):
            emb = EmbeddingVector(
                id=f"full-test-{i}",
                vector=[float(i+1)*0.1, float(i+1)*0.2, float(i+1)*0.3],
                text_chunk_id=f"full-chunk-{i}",
                model_name="test-model",
                created_at=datetime.now(),
                dimension=3
            )
            test_embeddings.append(emb)

        with patch('src.services.vector_storage.QdrantClient') as mock_client_class:
            mock_instance = Mock()

            # Track uploaded points
            uploaded_points = []

            def mock_upload_points(collection_name, points):
                nonlocal uploaded_points
                uploaded_points.extend(points)

            mock_instance.upload_points.side_effect = mock_upload_points

            # Mock retrieve method
            def mock_retrieve(collection_name, ids):
                for point in uploaded_points:
                    if point.id == ids[0]:
                        mock_record = Mock()
                        mock_record.id = point.id
                        mock_record.vector = point.vector
                        mock_record.payload = point.payload
                        return [mock_record]
                return []

            mock_instance.retrieve.side_effect = mock_retrieve

            # Mock search method
            def mock_search(collection_name, query_vector, limit):
                results = []
                for point in uploaded_points[:limit]:
                    mock_result = Mock()
                    mock_result.id = point.id
                    mock_result.vector = point.vector
                    mock_result.payload = point.payload
                    results.append(mock_result)
                return results

            mock_instance.search.side_effect = mock_search

            # Mock count method
            def mock_count(collection_name):
                mock_count_result = Mock()
                mock_count_result.count = len(uploaded_points)
                return mock_count_result

            mock_instance.count.side_effect = mock_count

            mock_client_class.return_value = mock_instance

            # Test full pipeline
            # 1. Store embeddings
            store_result = self.vector_storage.store_embeddings(test_embeddings)
            self.assertTrue(store_result)
            self.assertEqual(len(uploaded_points), 3)

            # 2. Retrieve specific embedding
            retrieved = self.vector_storage.retrieve_embedding("full-test-0")
            self.assertIsNotNone(retrieved)
            self.assertEqual(retrieved.id, "full-test-0")

            # 3. Search for similar embeddings
            search_results = self.vector_storage.search_similar([0.1, 0.2, 0.3], limit=2)
            self.assertEqual(len(search_results), 2)

            # 4. Count total embeddings
            count = self.vector_storage.count_embeddings()
            self.assertEqual(count, 3)