import React, { useState, useEffect } from 'react';
import RagQueryService from './RagQueryService';
import './RagQueryStyles.css';

/**
 * FloatingRagChat - A floating chat widget that can be placed on every page
 * Provides easy access to the RAG query functionality on all pages/chapters
 */
const FloatingRagChat = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [queryHistory, setQueryHistory] = useState([]);

  // Toggle chat window open/close
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Close chat window
  const closeChat = () => {
    setIsOpen(false);
  };

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

  // Handle Enter key press for submission
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      if (!loading && query.trim()) {
        handleSubmit(e);
      }
    }
  };

  return (
    <div className="floating-rag-chat">
      {/* Floating button to open chat */}
      {!isOpen && (
        <button
          className="floating-chat-button"
          onClick={toggleChat}
          aria-label="Open AI Assistant"
          title="Ask about the book content"
        >
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.35L2 22L7.65 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2ZM9 17C8.45 17 8 16.55 8 16C8 15.45 8.45 15 9 15C9.55 15 10 15.45 10 16C10 16.55 9.55 17 9 17ZM15 17C14.45 17 14 16.55 14 16C14 15.45 14.45 15 15 15C15.55 15 16 15.45 16 16C16 16.55 15.55 17 15 17ZM12 14C9.79 14 8 12.21 8 10C8 7.79 9.79 6 12 6C14.21 6 16 7.79 16 10C16 12.21 14.21 14 12 14Z" fill="currentColor"/>
          </svg>
        </button>
      )}

      {/* Chat window */}
      {isOpen && (
        <div className="floating-chat-window" role="dialog" aria-modal="true" aria-labelledby="chat-title">
          <div className="floating-chat-header">
            <h3 id="chat-title">AI Assistant</h3>
            <button
              className="close-chat-button"
              onClick={closeChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className="floating-chat-content">
            {/* Display previous query and response if available */}
            {response && (
              <div className="chat-response-display">
                <div className="rag-query-response">
                  <h4>Response</h4>
                  <div className="rag-response-content">
                    {response.response}
                  </div>

                  {response.sources && response.sources.length > 0 && (
                    <div className="rag-response-sources">
                      <h5>Sources:</h5>
                      <ul className="rag-sources-list">
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
                    <div className="rag-response-metadata">
                      Processed in {(response.metadata.processing_time / 1000).toFixed(2)} seconds
                    </div>
                  )}
                </div>
              </div>
            )}

            {/* Query form */}
            <form onSubmit={handleSubmit} className="rag-query-form" aria-label="RAG Query Form">
              <div className="rag-query-input-container">
                <textarea
                  id="floating-rag-query-input"
                  value={query}
                  onChange={handleInputChange}
                  onKeyPress={handleKeyPress}
                  placeholder="Ask about the book content..."
                  className="rag-query-input"
                  rows="3"
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

              <div className="chat-form-actions">
                <button
                  type="submit"
                  className="rag-query-submit-btn"
                  disabled={loading || !query.trim()}
                  aria-busy={loading}
                  aria-disabled={loading || !query.trim()}
                >
                  {loading ? 'Processing...' : 'Send'}
                </button>
              </div>
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

            {/* Recent queries */}
            {queryHistory.length > 0 && (
              <div className="rag-query-history" role="region" aria-labelledby="query-history-title">
                <h4 id="query-history-title">Recent Queries</h4>
                <div className="rag-history-list" aria-label="Query history">
                  {queryHistory.slice(0, 3).map((item, index) => (
                    <div key={index} className="rag-history-item">
                      <div className="rag-history-query" tabIndex="0">
                        <strong>Q:</strong> {item.query.substring(0, 50)}{item.query.length > 50 ? '...' : ''}
                      </div>
                    </div>
                  ))}
                </div>
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default FloatingRagChat;