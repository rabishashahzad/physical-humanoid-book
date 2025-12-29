import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from typing import List, Set
import time
import logging
from src.models.embedding import TextChunk, BookURL
from src.lib.config import Config
from src.services.text_processor import TextProcessor
from datetime import datetime


class CrawlerService:
    """Service for crawling deployed book URLs and extracting content."""

    def __init__(self):
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; BookEmbeddingsBot/1.0)'
        })
        self.logger = logging.getLogger(__name__)
        self.text_processor = TextProcessor()

    def discover_urls(self, base_url: str) -> List[str]:
        """Discover all publicly accessible book URLs from the base URL.

        Args:
            base_url: The base URL of the deployed book site

        Returns:
            List of discovered URLs
        """
        self.logger.info(f"Starting URL discovery for: {base_url}")
        all_urls = set()
        to_visit = [base_url]
        visited = set()

        # Limit to prevent infinite crawling
        max_pages = 100
        pages_crawled = 0

        while to_visit and pages_crawled < max_pages:
            current_url = to_visit.pop(0)

            if current_url in visited:
                continue

            visited.add(current_url)

            try:
                response = self.session.get(current_url, timeout=10)
                response.raise_for_status()

                soup = BeautifulSoup(response.text, 'html.parser')

                # Extract all internal links
                for link in soup.find_all('a', href=True):
                    href = link['href']
                    full_url = urljoin(current_url, href)

                    # Only add URLs from the same domain
                    if self._is_same_domain(base_url, full_url):
                        # Filter out non-content URLs
                        if self._is_content_url(full_url):
                            all_urls.add(full_url)
                            if full_url not in visited and full_url not in to_visit:
                                to_visit.append(full_url)

                pages_crawled += 1
                self.logger.info(f"Crawled {pages_crawled} pages, discovered {len(all_urls)} URLs so far")

            except requests.RequestException as e:
                self.logger.warning(f"Failed to crawl {current_url}: {str(e)}")
                continue

        self.logger.info(f"URL discovery completed. Found {len(all_urls)} URLs")
        return list(all_urls)

    def extract_content_from_url(self, url: str) -> List[TextChunk]:
        """Extract clean text content from a specific URL.

        Args:
            url: The URL to extract content from

        Returns:
            List of TextChunk objects containing the extracted content
        """
        try:
            self.logger.info(f"Extracting content from: {url}")
            response = self.session.get(url, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.text, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Try to find main content area (Docusaurus specific selectors)
            content_selectors = [
                'main',  # Main content area
                'article',  # Article content
                '.markdown',  # Docusaurus markdown content
                '.container',  # Container with content
                '.main-wrapper',  # Common wrapper
                '.theme-doc-markdown',  # Docusaurus specific
                '.docs-doc-id-*',  # Docusaurus document ID
                '[role="main"]',  # ARIA main role
            ]

            content_text = ""
            for selector in content_selectors:
                content_element = soup.select_one(selector)
                if content_element:
                    content_text = content_element.get_text(separator=' ', strip=True)
                    if content_text and len(content_text) > 100:  # Ensure substantial content
                        break

            # Fallback to body if no specific content found
            if not content_text or len(content_text) < 100:
                body = soup.find('body')
                if body:
                    content_text = body.get_text(separator=' ', strip=True)

            if not content_text:
                self.logger.warning(f"No content found at {url}")
                return []

            # Clean and normalize the text
            cleaned_content = self.text_processor.clean_text(content_text)

            if len(cleaned_content) < 50:  # Too little content to be useful
                self.logger.warning(f"Insufficient content at {url} ({len(cleaned_content)} chars)")
                return []

            # Create text chunks
            chunks = self.text_processor.chunk_text(cleaned_content, url)

            self.logger.info(f"Extracted {len(chunks)} chunks from {url}")
            return chunks

        except requests.RequestException as e:
            self.logger.error(f"Failed to extract content from {url}: {str(e)}")
            return []
        except Exception as e:
            self.logger.error(f"Unexpected error extracting content from {url}: {str(e)}")
            return []

    def _is_same_domain(self, base_url: str, url: str) -> bool:
        """Check if the URL is from the same domain as the base URL."""
        base_domain = urlparse(base_url).netloc
        url_domain = urlparse(url).netloc
        return base_domain == url_domain

    def _is_content_url(self, url: str) -> bool:
        """Check if the URL is likely to contain content (not navigation, etc.)."""
        # Filter out common non-content URLs
        non_content_patterns = [
            '/tag/', '/category/', '/author/', '/feed', '/rss',
            '.pdf', '.jpg', '.png', '.gif', '.zip', '.exe',
            '/admin/', '/login/', '/register/', '/api/',
            '/search', '/404', '/500'
        ]

        url_lower = url.lower()
        for pattern in non_content_patterns:
            if pattern in url_lower:
                return False

        # Only allow URLs that look like content pages
        return True