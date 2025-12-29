/**
 * RagQueryService - Specific service for query-related operations
 * Wraps the main RagApiService with query-specific functionality
 */

import RagApiService from '../../services/api/RagApiService';

class RagQueryService {
  /**
   * Submit a query to the RAG agent
   * @param {string} query - The user's query
   * @param {string} userId - Optional user identifier
   * @param {string} sessionId - Optional session identifier
   * @returns {Promise} Query response from the backend
   */
  static async submitQuery(query, userId = null, sessionId = null) {
    // Validate query length before sending
    if (!query || query.trim().length === 0) {
      throw new Error('Query cannot be empty');
    }

    if (query.length > 1000) {
      throw new Error('Query is too long. Maximum length is 1000 characters.');
    }

    try {
      return await RagApiService.queryAgent(query, userId, sessionId);
    } catch (error) {
      // Parse and handle error responses from the backend
      throw this.parseErrorResponse(error);
    }
  }

  /**
   * Parse error response from the backend API
   * @param {Error|Object} error - Error object from API call
   * @returns {Error} Formatted error with user-friendly message
   */
  static parseErrorResponse(error) {
    // If it's already a user-friendly error message, return as is
    if (error.message && !error.message.includes('Error:')) {
      return error;
    }

    // Handle different types of errors
    if (error.response) {
      // Server responded with error status
      const { status, data } = error.response;

      // Map backend error codes to user-friendly messages
      if (data && data.error) {
        switch (data.error.code) {
          case 'QUERY_TOO_LONG':
            return new Error('Query is too long. Please keep your question under 1000 characters.');
          case 'QUERY_EMPTY':
            return new Error('Query cannot be empty. Please enter a question.');
          case 'INVALID_FORMAT':
            return new Error('Invalid query format. Please try rephrasing your question.');
          case 'BACKEND_UNAVAILABLE':
            return new Error('The backend service is temporarily unavailable. Please try again later.');
          case 'PROCESSING_ERROR':
            return new Error('An error occurred while processing your query. Please try again.');
          case 'TIMEOUT_ERROR':
            return new Error('The query took too long to process. Please try again with a simpler question.');
          default:
            return new Error(data.error.message || `Server error (${status}): ${data.error.detail || 'An error occurred'}`);
        }
      } else {
        // Check for rate limiting (HTTP 429)
        if (status === 429) {
          const retryAfter = data.retryAfter || 'later';
          return new Error(`Rate limit exceeded. Please try again ${retryAfter}.`);
        }
        // Generic server error
        return new Error(`Server error (${status}): ${error.response.statusText}`);
      }
    } else if (error.request) {
      // Request was made but no response received
      return new Error('Network error: Unable to connect to the backend service. Please check your connection.');
    } else {
      // Something else happened
      return new Error(error.message || 'An unexpected error occurred while processing your query.');
    }
  }

  /**
   * Validate a query string
   * @param {string} query - Query to validate
   * @returns {Array} Array of validation errors, empty if valid
   */
  static validateQuery(query) {
    const errors = [];

    if (!query) {
      errors.push('Query is required');
    } else if (typeof query !== 'string') {
      errors.push('Query must be a string');
    } else if (query.trim().length === 0) {
      errors.push('Query must not be empty or whitespace-only');
    } else if (query.length > 1000) {
      errors.push('Query must be between 1 and 1000 characters');
    }

    return errors;
  }

  /**
   * Validate a response from the backend
   * @param {Object} response - Response object to validate
   * @returns {Array} Array of validation errors, empty if valid
   */
  static validateResponse(response) {
    const errors = [];

    if (!response) {
      errors.push('Response is required');
    } else {
      if (!response.response || typeof response.response !== 'string') {
        errors.push('Response must have a valid response string');
      }

      if (!response.sources || !Array.isArray(response.sources)) {
        errors.push('Response must have a valid sources array');
      }

      if (!response.queryId || typeof response.queryId !== 'string') {
        errors.push('Response must have a valid queryId string');
      }
    }

    return errors;
  }

  /**
   * Format response for display
   * @param {Object} response - Raw response from backend
   * @returns {Object} Formatted response object
   */
  static formatResponse(response) {
    return {
      content: response.response,
      sources: response.sources || [],
      confidence: response.confidence || null,
      queryId: response.queryId,
      metadata: response.metadata || {},
      timestamp: response.timestamp || new Date().toISOString()
    };
  }

  /**
   * Check if the service is healthy
   * @returns {Promise<boolean>} True if the service is healthy
   */
  static async checkHealth() {
    try {
      const health = await RagApiService.healthCheck();
      return health && health.status && health.status !== 'unhealthy';
    } catch (error) {
      console.error('Error checking service health:', error);
      return false;
    }
  }

  /**
   * Track query usage for analytics (optional)
   * @param {string} query - The query that was submitted
   * @param {Object} response - The response received
   * @param {Object} metadata - Additional metadata for tracking
   */
  static trackQueryUsage(query, response, metadata = {}) {
    // Optional analytics tracking - only if analytics provider is configured
    if (typeof window !== 'undefined' && window.gtag) {
      // Google Analytics tracking
      window.gtag('event', 'rag_query', {
        query_length: query.length,
        response_length: response ? response.response?.length : 0,
        has_sources: response ? response.sources?.length > 0 : false,
        processing_time: response?.metadata?.processing_time,
        ...metadata
      });
    }
    // Additional analytics providers can be added here
  }
}

export default RagQueryService;