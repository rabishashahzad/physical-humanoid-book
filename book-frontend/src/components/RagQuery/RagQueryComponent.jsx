import React, { useState } from 'react';
import RagQueryService from './RagQueryService';
import './RagQueryStyles.css';

/**
 * RagQueryComponent - A component for querying the RAG agent
 * Allows users to submit questions and receive AI-generated responses with source attribution
 */
const RagQueryComponent = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [queryHistory, setQueryHistory] = useState([]);

  /**
   * Handle form submission
   * @param {Event} e - Form submission event
   */
  const handleSubmit = async (e) => {
    e.preventDefault();

    // Validate query length
    if (!query.trim()) {
      setError('Query cannot be empty');
      return;
    }

    if (query.length > 1000) {
      setError('Query is too long. Maximum length is 1000 characters.');
      return;
    }

    setLoading(true);
    setError(null);

    try {
      // Query the RAG agent using the specific query service
      const result = await RagQueryService.submitQuery(query);

      // Validate the response
      const validationErrors = RagQueryService.validateResponse(result);
      if (validationErrors.length > 0) {
        throw new Error(`Invalid response format: ${validationErrors.join(', ')}`);
      }

      // Format the response for display
      const formattedResponse = RagQueryService.formatResponse(result);

      // Update response state
      setResponse(formattedResponse);

      // Add to query history
      setQueryHistory(prev => [
        {
          query: query,
          response: formattedResponse,
          timestamp: new Date().toISOString()
        },
        ...prev.slice(0, 9) // Keep only last 10 queries
      ]);

      // Clear the input field
      setQuery('');
    } catch (err) {
      console.error('Error querying RAG agent:', err);
      // Handle different types of errors with appropriate messages
      if (err.message) {
        setError(err.message);
      } else if (typeof err === 'string') {
        setError(err);
      } else {
        setError('An error occurred while processing your query. Please try again.');
      }
    } finally {
      setLoading(false);
    }
  };

  /**
   * Handle input change
   * @param {Event} e - Input change event
   */
  const handleInputChange = (e) => {
    setQuery(e.target.value);

    // Clear error when user starts typing
    if (error) {
      setError(null);
    }
  };

  return (
    <div className="rag-query-container" role="region" aria-labelledby="rag-query-title">
      <h3 id="rag-query-title">Ask about the Book Content</h3>

      <form onSubmit={handleSubmit} className="rag-query-form" aria-label="RAG Query Form">
        <div className="rag-query-input-container">
          <textarea
            id="rag-query-input"
            value={query}
            onChange={handleInputChange}
            placeholder="Enter your question about the book content..."
            className="rag-query-input"
            rows="4"
            disabled={loading}
            maxLength="1000"
            aria-describedby="query-character-count"
            aria-invalid={!!error}
            aria-required="true"
          />
          <div id="query-character-count" className="query-character-count" aria-live="polite">
            {query.length}/1000
          </div>
        </div>

        <button
          type="submit"
          className="rag-query-submit-btn"
          disabled={loading || !query.trim()}
          aria-busy={loading}
          aria-disabled={loading || !query.trim()}
        >
          {loading ? 'Processing...' : 'Ask Question'}
        </button>
      </form>

      {error && (
        <div className="rag-query-error" role="alert" aria-live="assertive">
          <strong>Error:</strong> {error}
        </div>
      )}

      {loading && (
        <div className="rag-query-loading" aria-live="polite">
          Processing your query...
        </div>
      )}

      {response && (
        <div className="rag-query-response" role="region" aria-labelledby="rag-response-title">
          <h4 id="rag-response-title">Response</h4>
          <div className="rag-response-content" tabIndex="0">
            {response.response}
          </div>

          {response.sources && response.sources.length > 0 && (
            <div className="rag-response-sources" aria-labelledby="sources-title">
              <h5 id="sources-title">Sources:</h5>
              <ul className="rag-sources-list" aria-label="List of sources">
                {response.sources.map((source, index) => (
                  <li key={index} className="rag-source-item">
                    <a
                      href={source.url}
                      target="_blank"
                      rel="noopener noreferrer"
                      className="rag-source-link"
                      aria-label={`Source: ${source.title} - ${source.section}`}
                    >
                      {source.title} - {source.section}
                    </a>
                    {source.confidence && (
                      <span className="rag-source-confidence" aria-label={`Confidence: ${source.confidence * 100}%`}>
                        (Confidence: {(source.confidence * 100).toFixed(1)}%)
                      </span>
                    )}
                  </li>
                ))}
              </ul>
            </div>
          )}

          {response.metadata && response.metadata.processing_time && (
            <div className="rag-response-metadata" aria-label={`Processed in ${(response.metadata.processing_time / 1000).toFixed(2)} seconds`}>
              Processed in {(response.metadata.processing_time / 1000).toFixed(2)} seconds
            </div>
          )}
        </div>
      )}

      {queryHistory.length > 0 && (
        <div className="rag-query-history" role="region" aria-labelledby="query-history-title">
          <h4 id="query-history-title">Recent Queries</h4>
          <div className="rag-history-list" aria-label="Query history">
            {queryHistory.map((item, index) => (
              <div key={index} className="rag-history-item">
                <div className="rag-history-query" tabIndex="0">
                  <strong>Q:</strong> {item.query}
                </div>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};

export default RagQueryComponent;