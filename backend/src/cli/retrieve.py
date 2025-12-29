#!/usr/bin/env python3
"""
Retrieval Pipeline Validation Script

This script validates the end-to-end retrieval pipeline by:
1. Generating embeddings for query text using Cohere
2. Performing similarity search against Qdrant collection
3. Returning results with metadata (URL, chunk index, text)
4. Validating relevance using predefined test queries
"""
import argparse
import time
import sys
from typing import List, Dict, Any, Optional

# Import from our modules
from ..lib.config import Config
from ..lib.cohere_client import cohere_client
from ..lib.qdrant_client import qdrant_client
from ..lib.logging import retrieval_logger
from ..lib.utils import (
    validate_text_length,
    validate_embedding_dimension,
    compare_embeddings_similarity
)


def retrieve_chunks(
    query_text: str,
    top_k: int = 5,
    threshold: float = 0.5
) -> Dict[str, Any]:
    """
    Retrieve chunks from Qdrant based on query text.

    Args:
        query_text: The query text to search for
        top_k: Number of results to return
        threshold: Minimum similarity score threshold

    Returns:
        Dictionary containing retrieval results and metadata
    """
    # Validate input
    if not validate_text_length(query_text):
        raise ValueError("Query text is too short (minimum 3 characters)")

    # Log start of retrieval
    start_time = retrieval_logger.log_retrieval_start(query_text)

    # Generate embedding for the query
    query_embedding = cohere_client.generate_embedding(query_text)
    retrieval_logger.log_embedding_generation(len(query_text))

    # Validate embedding dimension
    collection_info = qdrant_client.get_collection_info()
    expected_dimension = collection_info['vector_size']
    if not validate_embedding_dimension(query_embedding, expected_dimension):
        raise ValueError(
            f"Embedding dimension mismatch: expected {expected_dimension}, "
            f"got {len(query_embedding)}"
        )

    # Perform similarity search
    results = qdrant_client.search_similar(
        query_vector=query_embedding,
        top_k=top_k,
        threshold=threshold
    )

    # Calculate execution time
    end_time = time.time()
    execution_time = end_time - start_time.timestamp()

    # Log results
    retrieval_logger.log_retrieval_results(
        query_text, results, start_time, execution_time
    )

    # Format response
    response = {
        'query_text': query_text,
        'results': results,
        'execution_time': execution_time,
        'total_results': len(results),
        'model_info': cohere_client.get_model_info(),
        'collection_info': collection_info
    }

    return response


def validate_retrieval_results(
    query_text: str,
    results: List[Dict[str, Any]]
) -> Dict[str, Any]:
    """
    Validate that retrieval results meet quality criteria.

    Args:
        query_text: The original query text
        results: List of retrieved results

    Returns:
        Validation results dictionary
    """
    validation_details = {
        'has_results': len(results) > 0,
        'has_metadata': True,
        'metadata_complete': True,
        'relevance_check': True
    }

    # Check if results have complete metadata
    for result in results:
        if not all(key in result for key in ['text', 'url', 'chunk_index', 'score']):
            validation_details['metadata_complete'] = False
            break

    # Calculate relevance accuracy based on scores
    if results:
        avg_score = sum(r['score'] for r in results) / len(results)
        validation_details['avg_score'] = avg_score
        # Consider results valid if average score is above threshold
        validation_details['relevance_check'] = avg_score >= 0.5

    is_valid = (
        validation_details['has_results'] and
        validation_details['metadata_complete'] and
        validation_details['relevance_check']
    )

    validation_result = {
        'query': query_text,
        'is_valid': is_valid,
        'validation_details': validation_details
    }

    # Log validation result
    retrieval_logger.log_validation_result(
        query_text,
        is_valid,
        validation_details.get('avg_score')
    )

    return validation_result


def run_single_query(query_text: str, top_k: int = 5, threshold: float = 0.5):
    """Run a single query and display results."""
    try:
        # Retrieve chunks
        result = retrieve_chunks(query_text, top_k, threshold)

        # Validate results
        validation = validate_retrieval_results(query_text, result['results'])

        # Display results
        print(f"\nQuery: {query_text}")
        print(f"Execution time: {result['execution_time']:.3f}s")
        print(f"Total results: {result['total_results']}")
        print(f"Model: {result['model_info']['model']}")
        print(f"Collection: {result['collection_info']['name']}")
        print(f"Validation: {'PASS' if validation['is_valid'] else 'FAIL'}")

        print("\nResults:")
        for i, chunk in enumerate(result['results'], 1):
            print(f"\n{i}. Score: {chunk['score']:.3f}")
            print(f"   URL: {chunk['url']}")
            print(f"   Chunk Index: {chunk['chunk_index']}")
            print(f"   Text Preview: {chunk['text'][:100]}...")

    except Exception as e:
        retrieval_logger.log_error(f"Error during retrieval: {str(e)}")
        print(f"Error: {str(e)}", file=sys.stderr)
        sys.exit(1)


def run_batch_validation():
    """Run batch validation with predefined test queries."""
    test_queries = [
        "What does the book say about artificial intelligence?",
        "Explain the concept of machine learning",
        "How is data science used in practice?",
        "What are the ethical considerations in AI?",
        "Describe neural networks architecture"
    ]

    print("Running batch validation with predefined test queries...")
    all_valid = True
    total_time = 0

    for i, query in enumerate(test_queries, 1):
        print(f"\n{i}. Testing query: '{query}'")
        try:
            result = retrieve_chunks(query, top_k=3, threshold=0.3)
            validation = validate_retrieval_results(query, result['results'])
            is_valid = validation['is_valid']
            all_valid = all_valid and is_valid
            total_time += result['execution_time']

            status = "PASS" if is_valid else "FAIL"
            print(f"   Status: {status}")
            print(f"   Results: {result['total_results']}")
            print(f"   Time: {result['execution_time']:.3f}s")

        except Exception as e:
            print(f"   Status: ERROR - {str(e)}")
            all_valid = False

    print(f"\nBatch validation complete!")
    print(f"Overall status: {'PASS' if all_valid else 'FAIL'}")
    print(f"Total time: {total_time:.3f}s")
    print(f"Average time per query: {total_time/len(test_queries):.3f}s")


def main():
    """Main function to handle command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Validate retrieval pipeline against Qdrant collection"
    )
    parser.add_argument(
        '--query',
        type=str,
        help='Query text to retrieve similar chunks for'
    )
    parser.add_argument(
        '--top-k',
        type=int,
        default=5,
        help='Number of results to return (default: 5)'
    )
    parser.add_argument(
        '--threshold',
        type=float,
        default=0.5,
        help='Minimum similarity score threshold (default: 0.5)'
    )
    parser.add_argument(
        '--validate-all',
        action='store_true',
        help='Run batch validation with predefined test queries'
    )

    args = parser.parse_args()

    # Validate configuration
    config_errors = Config.validate()
    if config_errors:
        print("Configuration errors found:")
        for error in config_errors:
            print(f"  - {error}")
        sys.exit(1)

    # Validate Qdrant connection
    if not qdrant_client.validate_connection():
        print("Error: Cannot connect to Qdrant collection")
        sys.exit(1)

    if args.validate_all:
        run_batch_validation()
    elif args.query:
        run_single_query(args.query, args.top_k, args.threshold)
    else:
        # Default behavior - run a sample query
        sample_query = "What does the book say about artificial intelligence?"
        print("No query provided, running sample query...")
        run_single_query(sample_query, args.top_k, args.threshold)


if __name__ == "__main__":
    main()