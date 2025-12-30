import re
from typing import List
from src.models.embedding import TextChunk
import uuid
from datetime import datetime


class TextProcessor:
    """Service for text cleaning, normalization, and chunking."""

    def __init__(self):
        # Compile regex patterns for efficiency
        self.whitespace_pattern = re.compile(r'\s+')
        self.html_entity_pattern = re.compile(r'&[a-zA-Z]+;|&#\d+;')

    def clean_text(self, text: str) -> str:
        """Clean and normalize text content.

        Args:
            text: Raw text to clean

        Returns:
            Cleaned and normalized text
        """
        if not text:
            return ""

        # Replace HTML entities
        text = self.html_entity_pattern.sub(' ', text)

        # Normalize whitespace
        text = self.whitespace_pattern.sub(' ', text)

        # Remove excessive newlines (more than 2)
        text = re.sub(r'\n\s*\n\s*\n+', '\n\n', text)

        # Strip leading/trailing whitespace
        text = text.strip()

        return text

    def chunk_text(self, text: str, source_url: str) -> List[TextChunk]:
        """Chunk text into appropriately sized segments.

        Args:
            text: Text to chunk
            source_url: URL where the text was extracted from

        Returns:
            List of TextChunk objects
        """
        if not text:
            return []

        chunks = []
        chunk_size = 1000  # Default from config
        overlap = 100  # Default from config

        # Get values from config if available
        try:
            from src.lib.config import Config
            chunk_size = Config.CHUNK_SIZE
            overlap = Config.CHUNK_OVERLAP
        except ImportError:
            pass  # Use defaults if config not available

        start = 0
        chunk_index = 0

        while start < len(text):
            # Determine the end position
            end = start + chunk_size

            # If we're near the end, just take the rest
            if end >= len(text):
                end = len(text)
            else:
                # Try to break at sentence boundary
                sentence_end = text.rfind('.', start, end)
                if sentence_end != -1 and sentence_end > start + chunk_size // 2:
                    end = sentence_end + 1
                else:
                    # Try to break at word boundary
                    word_end = text.rfind(' ', start, end)
                    if word_end != -1 and word_end > start + chunk_size // 2:
                        end = word_end

            # Extract the chunk
            chunk_text = text[start:end].strip()

            if chunk_text:  # Only add non-empty chunks
                chunk = TextChunk(
                    id=str(uuid.uuid4()),
                    content=chunk_text,
                    source_url=source_url,
                    chunk_index=chunk_index,
                    created_at=datetime.now(),
                    word_count=len(chunk_text.split()),
                    char_count=len(chunk_text),
                    metadata={"source_type": "book_content", "processing_method": "text_chunker"}
                )
                chunks.append(chunk)

            # Move to the next position with overlap
            start = end - overlap if end < len(text) else end
            chunk_index += 1

        return chunks