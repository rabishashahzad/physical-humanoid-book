import cohere
from typing import List
import time
import logging
from src.models.embedding import TextChunk, EmbeddingVector
from src.lib.config import Config
import uuid
from datetime import datetime


class EmbeddingGenerator:
    """Service for generating embeddings using Cohere API."""

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.client = cohere.Client(Config.COHERE_API_KEY)
        self.model_name = "embed-english-v3.0"  # Using Cohere's latest embedding model

    def generate_embeddings(self, text_chunks: List[TextChunk]) -> List[EmbeddingVector]:
        """Generate embeddings for a list of text chunks.

        Args:
            text_chunks: List of TextChunk objects to generate embeddings for

        Returns:
            List of EmbeddingVector objects
        """
        if not text_chunks:
            return []

        embedding_vectors = []

        # Process in batches to respect API limits
        batch_size = 96  # Cohere's max batch size is 96
        for i in range(0, len(text_chunks), batch_size):
            batch = text_chunks[i:i + batch_size]
            batch_embeddings = self._generate_batch_embeddings(batch)
            embedding_vectors.extend(batch_embeddings)

        return embedding_vectors

    def _generate_batch_embeddings(self, text_chunks: List[TextChunk]) -> List[EmbeddingVector]:
        """Generate embeddings for a batch of text chunks.

        Args:
            text_chunks: List of TextChunk objects (max 96 per batch)

        Returns:
            List of EmbeddingVector objects
        """
        texts = [chunk.content for chunk in text_chunks]

        try:
            # Generate embeddings using Cohere
            response = self.client.embed(
                texts=texts,
                model=self.model_name,
                input_type="search_document"  # Appropriate for document search
            )

            embedding_vectors = []
            for i, embedding in enumerate(response.embeddings):
                text_chunk = text_chunks[i]

                embedding_vector = EmbeddingVector(
                    id=str(uuid.uuid4()),
                    vector=embedding,
                    text_chunk_id=text_chunk.id,
                    model_name=self.model_name,
                    created_at=datetime.now(),
                    dimension=len(embedding)
                )

                embedding_vectors.append(embedding_vector)

            self.logger.info(f"Generated embeddings for {len(text_chunks)} text chunks")
            return embedding_vectors

        except Exception as e:
            self.logger.error(f"Error generating embeddings: {str(e)}")
            # Implement retry logic with exponential backoff
            return self._retry_with_backoff(texts, text_chunks)

    def _retry_with_backoff(self, texts: List[str], text_chunks: List[TextChunk]) -> List[EmbeddingVector]:
        """Retry embedding generation with exponential backoff.

        Args:
            texts: List of text strings to embed
            text_chunks: Original TextChunk objects

        Returns:
            List of EmbeddingVector objects
        """
        max_retries = 3
        base_delay = 1  # seconds

        for attempt in range(max_retries):
            try:
                time.sleep(base_delay * (2 ** attempt))  # Exponential backoff

                response = self.client.embed(
                    texts=texts,
                    model=self.model_name,
                    input_type="search_document"
                )

                embedding_vectors = []
                for i, embedding in enumerate(response.embeddings):
                    text_chunk = text_chunks[i]

                    embedding_vector = EmbeddingVector(
                        id=str(uuid.uuid4()),
                        vector=embedding,
                        text_chunk_id=text_chunk.id,
                        model_name=self.model_name,
                        created_at=datetime.now(),
                        dimension=len(embedding)
                    )

                    embedding_vectors.append(embedding_vector)

                self.logger.info(f"Successfully generated embeddings after {attempt + 1} attempts")
                return embedding_vectors

            except Exception as e:
                self.logger.warning(f"Attempt {attempt + 1} failed: {str(e)}")
                if attempt == max_retries - 1:  # Last attempt
                    self.logger.error(f"All {max_retries} attempts failed for embedding generation")
                    # Return empty list or raise exception based on requirements
                    return []

        return []