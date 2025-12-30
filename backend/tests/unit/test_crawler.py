import unittest
from unittest.mock import Mock, patch
from src.services.crawler import CrawlerService
from src.models.embedding import TextChunk


class TestCrawlerService(unittest.TestCase):
    """Unit tests for CrawlerService."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.crawler = CrawlerService()

    @patch('src.services.crawler.requests.Session.get')
    def test_discover_urls(self, mock_get):
        """Test URL discovery functionality."""
        # Mock response for the base URL
        mock_response = Mock()
        mock_response.text = '''
        <html>
            <body>
                <a href="/page1">Page 1</a>
                <a href="/page2">Page 2</a>
                <a href="https://external.com">External</a>
            </body>
        </html>
        '''
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response

        urls = self.crawler.discover_urls("https://example.com")

        # Should only return internal URLs
        self.assertIn("https://example.com/page1", urls)
        self.assertIn("https://example.com/page2", urls)
        # External URL should not be included
        internal_urls = [url for url in urls if "example.com" in url]
        self.assertEqual(len(internal_urls), 2)

    @patch('src.services.crawler.requests.Session.get')
    def test_extract_content_from_url_success(self, mock_get):
        """Test content extraction from a URL."""
        # Mock response
        mock_response = Mock()
        mock_response.text = '''
        <html>
            <head><title>Test Page</title></head>
            <body>
                <main>
                    <h1>Test Content</h1>
                    <p>This is the main content of the page.</p>
                    <p>It has multiple paragraphs.</p>
                </main>
            </body>
        </html>
        '''
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response

        chunks = self.crawler.extract_content_from_url("https://example.com/test")

        # Should return at least one chunk
        self.assertGreater(len(chunks), 0)
        for chunk in chunks:
            self.assertIsInstance(chunk, TextChunk)
            self.assertGreater(len(chunk.content), 0)
            self.assertEqual(chunk.source_url, "https://example.com/test")

    @patch('src.services.crawler.requests.Session.get')
    def test_extract_content_from_url_request_error(self, mock_get):
        """Test content extraction when request fails."""
        # Mock a request exception
        mock_get.side_effect = Exception("Network error")

        chunks = self.crawler.extract_content_from_url("https://example.com/test")

        # Should return empty list on error
        self.assertEqual(chunks, [])

    def test_is_same_domain(self):
        """Test domain comparison."""
        result = self.crawler._is_same_domain("https://example.com", "https://example.com/page")
        self.assertTrue(result)

        result = self.crawler._is_same_domain("https://example.com", "https://other.com/page")
        self.assertFalse(result)

    def test_is_content_url(self):
        """Test content URL filtering."""
        # Valid content URLs
        self.assertTrue(self.crawler._is_content_url("https://example.com/page"))
        self.assertTrue(self.crawler._is_content_url("https://example.com/docs/guide"))

        # Non-content URLs
        self.assertFalse(self.crawler._is_content_url("https://example.com/api/data"))
        self.assertFalse(self.crawler._is_content_url("https://example.com/image.jpg"))
        self.assertFalse(self.crawler._is_content_url("https://example.com/admin"))