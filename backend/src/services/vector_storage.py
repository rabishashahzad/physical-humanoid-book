from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional
import logging
from src.models.embedding import EmbeddingVector
from src.lib.config import Config
import uuid


class VectorStorage:
    """Service for storing embeddings in Qdrant Cloud."""

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.client = QdrantClient(
            url=Config.QDRANT_HOST,
            api_key=Config.QDRANT_API_KEY,
        )
        self.collection_name = Config.QDRANT_COLLECTION_NAME

    def store_embeddings(self, embedding_vectors: List[EmbeddingVector]) -> bool:
        """Store embedding vectors in Qdrant collection.

        Args:
            embedding_vectors: List of EmbeddingVector objects to store

        Returns:
            True if storage was successful, False otherwise
        """
        if not embedding_vectors:
            self.logger.info("No embeddings to store")
            return True

        try:
            # Ensure collection exists
            self._ensure_collection_exists()

            # Prepare points for insertion
            points = []
            for embedding_vector in embedding_vectors:
                point = models.PointStruct(
                    id=embedding_vector.id,
                    vector=embedding_vector.vector,
                    payload={
                        "text_chunk_id": embedding_vector.text_chunk_id,
                        "model_name": embedding_vector.model_name,
                        "created_at": embedding_vector.created_at.isoformat(),
                        "dimension": embedding_vector.dimension
                    }
                )
                points.append(point)

            # Upload points to Qdrant
            self.client.upload_points(
                collection_name=self.collection_name,
                points=points
            )

            self.logger.info(f"Successfully stored {len(embedding_vectors)} embeddings in Qdrant collection: {self.collection_name}")
            return True

        except Exception as e:
            self.logger.error(f"Error storing embeddings in Qdrant: {str(e)}")
            return False

    def retrieve_embedding(self, embedding_id: str) -> Optional[EmbeddingVector]:
        """Retrieve a specific embedding by ID.

        Args:
            embedding_id: ID of the embedding to retrieve

        Returns:
            EmbeddingVector if found, None otherwise
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[embedding_id]
            )

            if not records:
                return None

            record = records[0]
            return EmbeddingVector(
                id=record.id,
                vector=record.vector,
                text_chunk_id=record.payload.get("text_chunk_id"),
                model_name=record.payload.get("model_name"),
                created_at=record.payload.get("created_at"),
                dimension=record.payload.get("dimension", len(record.vector))
            )

        except Exception as e:
            self.logger.error(f"Error retrieving embedding {embedding_id} from Qdrant: {str(e)}")
            return None

    def search_similar(self, query_vector: List[float], limit: int = 10) -> List[EmbeddingVector]:
        """Search for similar embeddings using vector similarity.

        Args:
            query_vector: The vector to search for similar embeddings
            limit: Maximum number of results to return

        Returns:
            List of similar EmbeddingVector objects
        """
        try:
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )

            embedding_vectors = []
            for result in search_results:
                embedding_vector = EmbeddingVector(
                    id=result.id,
                    vector=result.vector,
                    text_chunk_id=result.payload.get("text_chunk_id"),
                    model_name=result.payload.get("model_name"),
                    created_at=result.payload.get("created_at"),
                    dimension=result.payload.get("dimension", len(result.vector))
                )
                embedding_vectors.append(embedding_vector)

            return embedding_vectors

        except Exception as e:
            self.logger.error(f"Error searching for similar embeddings in Qdrant: {str(e)}")
            return []

    def _ensure_collection_exists(self):
        """Ensure the Qdrant collection exists with proper configuration."""
        try:
            # Try to get collection info to check if it exists
            self.client.get_collection(self.collection_name)
            self.logger.info(f"Collection {self.collection_name} already exists")
        except Exception:
            # Collection doesn't exist, create it
            self.logger.info(f"Creating collection {self.collection_name}")

            # Get dimension from a sample embedding if available
            dimension = 1024  # Default Cohere embedding dimension

            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=dimension,
                    distance=models.Distance.COSINE
                )
            )

            self.logger.info(f"Created collection {self.collection_name} with dimension {dimension}")

    def count_embeddings(self) -> int:
        """Count the total number of embeddings in the collection.

        Returns:
            Number of embeddings in the collection
        """
        try:
            count_result = self.client.count(
                collection_name=self.collection_name
            )
            return count_result.count
        except Exception as e:
            self.logger.error(f"Error counting embeddings in Qdrant: {str(e)}")
            return 0