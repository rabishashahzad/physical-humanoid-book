import unittest
from unittest.mock import Mock, patch, MagicMock
from src.services.embedding_generator import EmbeddingGenerator
from src.models.embedding import TextChunk
from datetime import datetime


class TestEmbeddingGenerator(unittest.TestCase):
    """Unit tests for EmbeddingGenerator."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        with patch('src.services.embedding_generator.cohere.Client'):
            self.embedding_gen = EmbeddingGenerator()

    def test_generate_embeddings_empty_input(self):
        """Test generating embeddings with empty input."""
        result = self.embedding_gen.generate_embeddings([])
        self.assertEqual(result, [])

    @patch('src.services.embedding_generator.cohere.Client')
    def test_generate_embeddings_success(self, mock_cohere_client):
        """Test successful embedding generation."""
        # Mock the Cohere client response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        mock_cohere_client.return_value.embed.return_value = mock_response

        # Create test text chunks
        chunk1 = TextChunk(
            id="test1",
            content="This is the first text chunk.",
            source_url="https://example.com/page1",
            chunk_index=0,
            created_at=datetime.now(),
            word_count=6,
            char_count=30,
            metadata={}
        )
        chunk2 = TextChunk(
            id="test2",
            content="This is the second text chunk.",
            source_url="https://example.com/page2",
            chunk_index=1,
            created_at=datetime.now(),
            word_count=6,
            char_count=31,
            metadata={}
        )

        chunks = [chunk1, chunk2]

        with patch('src.services.embedding_generator.cohere.Client') as mock_client:
            mock_client.return_value = Mock()
            mock_client.return_value.embed.return_value = mock_response

            result = self.embedding_gen.generate_embeddings(chunks)

        # Should return 2 embedding vectors
        self.assertEqual(len(result), 2)
        for i, embedding in enumerate(result):
            self.assertEqual(embedding.text_chunk_id, chunks[i].id)
            self.assertEqual(len(embedding.vector), 3)  # 3 dimensions in mock
            self.assertEqual(embedding.model_name, "embed-english-v3.0")

    @patch('src.services.embedding_generator.cohere.Client')
    def test_generate_embeddings_batch_processing(self, mock_cohere_client):
        """Test embedding generation with batch processing."""
        # Mock the Cohere client response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2]]  # Single embedding per call
        mock_cohere_client.return_value.embed.return_value = mock_response

        # Create more chunks than batch size to test batching
        chunks = []
        for i in range(100):  # More than batch size of 96
            chunk = TextChunk(
                id=f"test{i}",
                content=f"This is text chunk {i}.",
                source_url="https://example.com/page",
                chunk_index=i,
                created_at=datetime.now(),
                word_count=5,
                char_count=20 + i,
                metadata={}
            )
            chunks.append(chunk)

        with patch('src.services.embedding_generator.cohere.Client') as mock_client:
            mock_client.return_value = Mock()
            mock_client.return_value.embed.return_value = mock_response

            result = self.embedding_gen.generate_embeddings(chunks)

        # Should return 100 embedding vectors
        self.assertEqual(len(result), 100)

    @patch('src.services.embedding_generator.cohere.Client')
    def test_generate_embeddings_with_retry(self, mock_cohere_client):
        """Test embedding generation with retry logic."""
        # Mock the Cohere client to fail initially then succeed
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]]
        mock_cohere_instance = Mock()
        mock_cohere_instance.embed.side_effect = [Exception("API Error"), mock_response]
        mock_cohere_client.return_value = mock_cohere_instance

        chunk = TextChunk(
            id="test1",
            content="This is a test chunk.",
            source_url="https://example.com/page1",
            chunk_index=0,
            created_at=datetime.now(),
            word_count=6,
            char_count=23,
            metadata={}
        )

        with patch('time.sleep'):  # Mock sleep to speed up test
            result = self.embedding_gen.generate_embeddings([chunk])

        # Should return 1 embedding vector after retry
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].text_chunk_id, "test1")