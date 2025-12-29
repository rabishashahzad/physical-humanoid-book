import unittest
from unittest.mock import Mock, patch
from src.services.embedding_generator import EmbeddingGenerator
from src.models.embedding import TextChunk
from src.services.text_processor import TextProcessor
from datetime import datetime


class TestEmbeddingGeneratorIntegration(unittest.TestCase):
    """Integration tests for EmbeddingGenerator with other components."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        with patch('src.services.embedding_generator.cohere.Client'):
            self.embedding_gen = EmbeddingGenerator()
        self.text_processor = TextProcessor()

    def test_text_processor_embedding_generator_integration(self):
        """Test integration between text processor and embedding generator."""
        # Create sample text and process it through text processor
        sample_text = "This is a sample text for testing the integration between text processor and embedding generator. " * 10
        source_url = "https://example.com/test-page"

        # Process through text processor to create chunks
        text_chunks = self.text_processor.chunk_text(sample_text, source_url)

        # Verify that text processor created chunks
        self.assertGreater(len(text_chunks), 0)
        for chunk in text_chunks:
            self.assertLessEqual(len(chunk.content), 1000 + 100)  # chunk_size + overlap
            self.assertEqual(chunk.source_url, source_url)

        # Now test that embedding generator can process these chunks
        # Mock the Cohere response
        mock_embeddings = [[0.1, 0.2, 0.3]] * len(text_chunks)  # Create embeddings for each chunk

        with patch('src.services.embedding_generator.cohere.Client') as mock_client:
            mock_response = Mock()
            mock_response.embeddings = mock_embeddings
            mock_client.return_value.embed.return_value = mock_response

            embedding_vectors = self.embedding_gen.generate_embeddings(text_chunks)

        # Verify that embeddings were generated for all chunks
        self.assertEqual(len(embedding_vectors), len(text_chunks))
        for i, embedding in enumerate(embedding_vectors):
            self.assertEqual(embedding.text_chunk_id, text_chunks[i].id)
            self.assertEqual(len(embedding.vector), 3)  # 3 dimensions in mock
            self.assertEqual(embedding.model_name, "embed-english-v3.0")

    def test_embedding_generation_preserves_chunk_metadata(self):
        """Test that embedding generation preserves original chunk metadata."""
        # Create a text chunk with specific properties
        original_chunk = TextChunk(
            id="test-chunk-123",
            content="This is a test content that should be embedded properly.",
            source_url="https://example.com/source-page",
            chunk_index=0,
            created_at=datetime.now(),
            word_count=12,
            char_count=60,
            metadata={"section": "introduction", "category": "technical"}
        )

        # Mock the Cohere response
        mock_embeddings = [[0.5, 0.6, 0.7]]

        with patch('src.services.embedding_generator.cohere.Client') as mock_client:
            mock_response = Mock()
            mock_response.embeddings = mock_embeddings
            mock_client.return_value.embed.return_value = mock_response

            embedding_vectors = self.embedding_gen.generate_embeddings([original_chunk])

        # Verify the embedding was created with correct reference to original chunk
        self.assertEqual(len(embedding_vectors), 1)
        embedding = embedding_vectors[0]

        # Verify the embedding references the original chunk
        self.assertEqual(embedding.text_chunk_id, original_chunk.id)
        self.assertEqual(len(embedding.vector), 3)
        self.assertEqual(embedding.model_name, "embed-english-v3.0")

    def test_multiple_chunks_embedding_consistency(self):
        """Test that multiple chunks are embedded consistently."""
        # Create multiple text chunks
        chunks = []
        for i in range(5):
            chunk = TextChunk(
                id=f"test-chunk-{i}",
                content=f"This is test content number {i} for consistency testing.",
                source_url="https://example.com/test-page",
                chunk_index=i,
                created_at=datetime.now(),
                word_count=10,
                char_count=50 + i,
                metadata={"batch": "test", "index": str(i)}
            )
            chunks.append(chunk)

        # Mock embeddings for all chunks
        mock_embeddings = [[0.1 + i*0.1, 0.2 + i*0.1, 0.3 + i*0.1] for i in range(5)]

        with patch('src.services.embedding_generator.cohere.Client') as mock_client:
            mock_response = Mock()
            mock_response.embeddings = mock_embeddings
            mock_client.return_value.embed.return_value = mock_response

            embedding_vectors = self.embedding_gen.generate_embeddings(chunks)

        # Verify all chunks were processed
        self.assertEqual(len(embedding_vectors), 5)
        for i, embedding in enumerate(embedding_vectors):
            self.assertEqual(embedding.text_chunk_id, chunks[i].id)
            self.assertEqual(len(embedding.vector), 3)
            # Verify each embedding is unique
            self.assertAlmostEqual(embedding.vector[0], 0.1 + i*0.1, places=1)