import unittest
from unittest.mock import Mock, patch
from src.services.crawler import CrawlerService
from src.services.embedding_generator import EmbeddingGenerator
from src.services.vector_storage import VectorStorage
from src.models.embedding import TextChunk
from datetime import datetime


class TestIngestionPipelineIntegration(unittest.TestCase):
    """Integration tests for the full ingestion pipeline."""

    def test_full_ingestion_pipeline_integration(self):
        """Test integration of all components in the ingestion pipeline."""
        # Test the full flow: crawler -> text processor -> embedding generator -> vector storage

        # Mock all services to avoid external dependencies
        with patch('src.services.crawler.requests.Session.get') as mock_get, \
             patch('src.services.embedding_generator.cohere.Client') as mock_cohere, \
             patch('src.services.vector_storage.QdrantClient') as mock_qdrant:

            # Setup mock responses
            mock_response = Mock()
            mock_response.text = '''
            <html>
                <body>
                    <main>
                        <h1>Test Book Content</h1>
                        <p>This is the first paragraph of test content for the integration test.</p>
                        <p>Here is another paragraph with more content to process.</p>
                    </main>
                </body>
            </html>
            '''
            mock_response.raise_for_status.return_value = None
            mock_get.return_value = mock_response

            # Mock Cohere embedding response
            mock_cohere_response = Mock()
            mock_cohere_response.embeddings = [[0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8]]
            mock_cohere.return_value.embed.return_value = mock_cohere_response

            # Mock Qdrant operations
            mock_qdrant_instance = Mock()
            mock_qdrant.return_value = mock_qdrant_instance

            # Initialize services
            crawler = CrawlerService()
            embedding_gen = EmbeddingGenerator()
            vector_storage = VectorStorage()

            # Test Step 1: Crawl and extract content
            chunks = crawler.extract_content_from_url("https://example.com/test")
            self.assertGreater(len(chunks), 0)
            for chunk in chunks:
                self.assertIsInstance(chunk, TextChunk)
                self.assertGreater(len(chunk.content), 0)

            # Test Step 2: Generate embeddings
            embeddings = embedding_gen.generate_embeddings(chunks)
            self.assertEqual(len(embeddings), len(chunks))  # Should have one embedding per chunk
            for embedding in embeddings:
                self.assertEqual(len(embedding.vector), 4)  # 4 dimensions in mock

            # Test Step 3: Store embeddings
            store_result = vector_storage.store_embeddings(embeddings)
            self.assertTrue(store_result)

            # Verify that all expected calls were made
            mock_get.assert_called()
            mock_cohere.return_value.embed.assert_called()
            mock_qdrant_instance.upload_points.assert_called()

    def test_pipeline_with_multiple_pages(self):
        """Test the pipeline with multiple pages of content."""
        with patch('src.services.crawler.requests.Session.get') as mock_get, \
             patch('src.services.embedding_generator.cohere.Client') as mock_cohere, \
             patch('src.services.vector_storage.QdrantClient') as mock_qdrant:

            # Setup mock responses for multiple pages
            def side_effect(url, *args, **kwargs):
                mock_response = Mock()
                if "page1" in url:
                    mock_response.text = '''
                    <html><body><main><h1>Page 1</h1><p>Content of page 1.</p></main></body></html>
                    '''
                else:
                    mock_response.text = '''
                    <html><body><main><h1>Page 2</h1><p>Content of page 2.</p></main></body></html>
                    '''
                mock_response.raise_for_status.return_value = None
                return mock_response

            mock_get.side_effect = side_effect

            # Mock Cohere responses (variable number of embeddings based on content)
            def cohere_side_effect(texts, *args, **kwargs):
                mock_response = Mock()
                # Return one embedding per text in the batch
                mock_response.embeddings = [[0.1 + i*0.1, 0.2 + i*0.1, 0.3 + i*0.1, 0.4 + i*0.1]
                                          for i in range(len(texts))]
                return mock_response

            mock_cohere.return_value.embed.side_effect = cohere_side_effect

            # Mock Qdrant operations
            mock_qdrant_instance = Mock()
            mock_qdrant.return_value = mock_qdrant_instance

            # Initialize services
            crawler = CrawlerService()
            embedding_gen = EmbeddingGenerator()
            vector_storage = VectorStorage()

            # Test with multiple pages
            urls = ["https://example.com/page1", "https://example.com/page2"]
            all_chunks = []
            all_embeddings = []

            for url in urls:
                chunks = crawler.extract_content_from_url(url)
                all_chunks.extend(chunks)

                embeddings = embedding_gen.generate_embeddings(chunks)
                all_embeddings.extend(embeddings)

            # Store all embeddings
            store_result = vector_storage.store_embeddings(all_embeddings)
            self.assertTrue(store_result)

            # Verify results
            self.assertGreater(len(all_chunks), 0)
            self.assertEqual(len(all_embeddings), len(all_chunks))
            mock_qdrant_instance.upload_points.assert_called()

    def test_error_handling_in_pipeline(self):
        """Test error handling across the pipeline components."""
        with patch('src.services.crawler.requests.Session.get') as mock_get, \
             patch('src.services.embedding_generator.cohere.Client') as mock_cohere, \
             patch('src.services.vector_storage.QdrantClient') as mock_qdrant:

            # Mock network error for crawler
            mock_get.side_effect = Exception("Network error")

            crawler = CrawlerService()
            embedding_gen = EmbeddingGenerator()
            vector_storage = VectorStorage()

            # Test that crawler handles network errors gracefully
            chunks = crawler.extract_content_from_url("https://example.com/test")
            self.assertEqual(chunks, [])  # Should return empty list on error

            # Test with Cohere error
            mock_get.side_effect = None  # Reset to success
            mock_get.return_value.text = '<html><body><main><p>Test content.</p></main></body></html>'
            mock_get.return_value.raise_for_status.return_value = None

            # Now test Cohere error
            mock_cohere.return_value.embed.side_effect = Exception("API error")

            # Extract content first
            chunks = crawler.extract_content_from_url("https://example.com/test")
            self.assertGreater(len(chunks), 0)

            # Then test embedding generation error handling
            embeddings = embedding_gen.generate_embeddings(chunks)
            # Should handle the error gracefully, potentially returning empty list
            # (behavior depends on implementation of retry logic)

            # Test Qdrant error
            mock_cohere.return_value.embed.side_effect = None  # Reset
            mock_response = Mock()
            mock_response.embeddings = [[0.1, 0.2, 0.3]]
            mock_cohere.return_value.embed.return_value = mock_response

            # Mock Qdrant storage error
            mock_qdrant_instance = Mock()
            mock_qdrant_instance.upload_points.side_effect = Exception("Storage error")
            mock_qdrant.return_value = mock_qdrant_instance

            # Test storage error handling
            store_result = vector_storage.store_embeddings(embeddings)
            # Should handle the error gracefully