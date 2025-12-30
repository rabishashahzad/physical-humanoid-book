import argparse
import sys
from typing import Optional

from src.lib.config import Config
from src.services.crawler import CrawlerService
from src.services.embedding_generator import EmbeddingGenerator
from src.services.vector_storage import VectorStorage


def main():
    """Main entry point for the book embeddings ingestion pipeline."""
    parser = argparse.ArgumentParser(description="Book Embeddings Ingestion Pipeline")
    parser.add_argument("--base-url", help="Base URL of the deployed book site")
    parser.add_argument("--collection-name", help="Name of the Qdrant collection")
    parser.add_argument("--chunk-size", type=int, help="Size of text chunks in characters")
    parser.add_argument("--chunk-overlap", type=int, help="Overlap between chunks in characters")
    parser.add_argument("--dry-run", action="store_true", help="Run without storing embeddings")
    parser.add_argument("--limit", type=int, help="Limit number of pages to process")

    args = parser.parse_args()

    # Validate configuration
    config_errors = Config.validate()
    if config_errors:
        print("Configuration errors found:")
        for error in config_errors:
            print(f"  - {error}")
        sys.exit(1)

    # Override config with command line arguments if provided
    if args.base_url:
        Config.BOOK_BASE_URL = args.base_url
    if args.collection_name:
        Config.QDRANT_COLLECTION_NAME = args.collection_name
    if args.chunk_size:
        Config.CHUNK_SIZE = args.chunk_size
    if args.chunk_overlap:
        Config.CHUNK_OVERLAP = args.chunk_overlap

    print(f"Starting ingestion pipeline for: {Config.BOOK_BASE_URL}")

    try:
        # Initialize services
        crawler = CrawlerService()
        embedding_generator = EmbeddingGenerator()
        vector_storage = VectorStorage()

        # Execute the pipeline
        print("Step 1: Discovering URLs...")
        urls = crawler.discover_urls(Config.BOOK_BASE_URL)
        print(f"Discovered {len(urls)} URLs to process")

        if args.limit:
            urls = urls[:args.limit]
            print(f"Limiting to first {len(urls)} URLs")

        print("Step 2: Extracting content...")
        text_chunks = []
        for i, url in enumerate(urls):
            print(f"Processing page {i+1}/{len(urls)}: {url}")
            chunks = crawler.extract_content_from_url(url)
            text_chunks.extend(chunks)
            print(f"  Extracted {len(chunks)} text chunks")

        print(f"Step 3: Generating embeddings for {len(text_chunks)} chunks...")
        embedding_vectors = embedding_generator.generate_embeddings(text_chunks)
        print(f"Generated {len(embedding_vectors)} embeddings")

        if not args.dry_run:
            print(f"Step 4: Storing embeddings in Qdrant collection: {Config.QDRANT_COLLECTION_NAME}")
            success = vector_storage.store_embeddings(embedding_vectors)
            if success:
                print("Embeddings stored successfully")
                print(f"Total embeddings in collection: {vector_storage.count_embeddings()}")
            else:
                print("Failed to store embeddings")
                sys.exit(1)
        else:
            print("Step 4: Dry run - skipping storage")

        # Add basic search test to validate storage
        if not args.dry_run and embedding_vectors:
            print("Step 5: Validating storage with a basic search...")
            # Perform a simple search to verify embeddings are queryable
            sample_vector = embedding_vectors[0].vector
            similar_embeddings = vector_storage.search_similar(sample_vector, limit=3)
            print(f"Found {len(similar_embeddings)} similar embeddings for validation")

        print("Pipeline completed successfully!")

    except Exception as e:
        print(f"Pipeline failed with error: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()