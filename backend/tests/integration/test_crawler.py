import unittest
from src.services.crawler import CrawlerService
from src.models.embedding import TextChunk


class TestCrawlerIntegration(unittest.TestCase):
    """Integration tests for CrawlerService with TextProcessor."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.crawler = CrawlerService()

    def test_crawler_text_processor_integration(self):
        """Test integration between crawler and text processor."""
        # Mock HTML content with various elements
        html_content = '''
        <html>
            <head><title>Test Page</title></head>
            <body>
                <main>
                    <h1>Introduction</h1>
                    <p>This is the first paragraph with some content.</p>
                    <p>Here is another paragraph with more content. This one is longer and contains more text to test the chunking functionality.</p>
                    <div class="sidebar">This should be filtered out.</div>
                    <script>console.log("This should be removed");</script>
                    <style>.hidden { display: none; }</style>
                </main>
            </body>
        </html>
        '''

        # Test the extract_content_from_url method with mocked response
        with unittest.mock.patch('src.services.crawler.requests.Session.get') as mock_get:
            mock_response = unittest.mock.Mock()
            mock_response.text = html_content
            mock_response.raise_for_status.return_value = None
            mock_get.return_value = mock_response

            chunks = self.crawler.extract_content_from_url("https://example.com/test")

            # Verify that chunks were created
            self.assertGreater(len(chunks), 0)
            for chunk in chunks:
                self.assertIsInstance(chunk, TextChunk)
                self.assertGreater(len(chunk.content), 0)
                # Ensure content doesn't contain script/style tags
                self.assertNotIn("console.log", chunk.content)
                self.assertNotIn(".hidden", chunk.content)
                self.assertEqual(chunk.source_url, "https://example.com/test")

    def test_content_extraction_preserves_meaning(self):
        """Test that content extraction preserves semantic meaning."""
        html_content = '''
        <html>
            <body>
                <article class="markdown">
                    <h1>Advanced Robotics</h1>
                    <p>Robotics is an interdisciplinary branch of engineering and science that includes mechanical engineering, electrical engineering, computer science, and others.</p>
                    <p>The goal of robotics is to design and create machines that can substitute for humans.</p>
                    <section>
                        <h2>Types of Robots</h2>
                        <p>Industrial robots are widely used in manufacturing.</p>
                        <p>Service robots assist humans in their daily routines.</p>
                    </section>
                </article>
            </body>
        </html>
        '''

        with unittest.mock.patch('src.services.crawler.requests.Session.get') as mock_get:
            mock_response = unittest.mock.Mock()
            mock_response.text = html_content
            mock_response.raise_for_status.return_value = None
            mock_get.return_value = mock_response

            chunks = self.crawler.extract_content_from_url("https://example.com/robotics")

            # Should extract meaningful content
            self.assertGreater(len(chunks), 0)
            full_content = " ".join([chunk.content for chunk in chunks])

            # Verify important content is preserved
            self.assertIn("Robotics", full_content)
            self.assertIn("engineering", full_content)
            self.assertIn("machines", full_content)
            self.assertIn("Industrial robots", full_content)