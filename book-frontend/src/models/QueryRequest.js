/**
 * QueryRequest Model
 * Represents a user query request to the RAG backend
 * Based on the data-model.md specification
 */

class QueryRequest {
  /**
   * Create a new QueryRequest
   * @param {string} query - The user's text query/question (required)
   * @param {string} [userId] - Identifier for the user making the request (optional)
   * @param {string} [sessionId] - Session identifier for tracking conversation context (optional)
   * @param {Object} [context] - Additional context information for the query (optional)
   */
  constructor(query, userId = null, sessionId = null, context = null) {
    this.query = query;
    this.userId = userId;
    this.sessionId = sessionId;
    this.context = context;
    this.timestamp = new Date().toISOString();
  }

  /**
   * Validate the query request
   * @returns {Array} Array of validation errors, empty if valid
   */
  validate() {
    const errors = [];

    // Validate query field
    if (!this.query) {
      errors.push('Query is required');
    } else if (typeof this.query !== 'string') {
      errors.push('Query must be a string');
    } else if (this.query.trim().length === 0) {
      errors.push('Query must not be empty or whitespace-only');
    } else if (this.query.length > 1000) {
      errors.push('Query must be between 1 and 1000 characters');
    }

    // Validate userId format if provided
    if (this.userId && typeof this.userId !== 'string') {
      errors.push('userId must be a string if provided');
    }

    // Validate sessionId format if provided
    if (this.sessionId && typeof this.sessionId !== 'string') {
      errors.push('sessionId must be a string if provided');
    }

    // Validate timestamp format
    if (this.timestamp) {
      const timestampDate = new Date(this.timestamp);
      if (isNaN(timestampDate.getTime())) {
        errors.push('timestamp must be in ISO 8601 format');
      }
    }

    // Validate context if provided
    if (this.context && typeof this.context !== 'object') {
      errors.push('context must be an object if provided');
    }

    return errors;
  }

  /**
   * Convert the request to a plain object for API transmission
   * @returns {Object} Plain object representation
   */
  toObject() {
    const obj = {
      query: this.query,
    };

    if (this.userId) {
      obj.userId = this.userId;
    }

    if (this.sessionId) {
      obj.sessionId = this.sessionId;
    }

    if (this.context) {
      obj.context = this.context;
    }

    if (this.timestamp) {
      obj.timestamp = this.timestamp;
    }

    return obj;
  }

  /**
   * Create a QueryRequest from a plain object
   * @param {Object} obj - Plain object to convert
   * @returns {QueryRequest} New QueryRequest instance
   */
  static fromObject(obj) {
    return new QueryRequest(
      obj.query,
      obj.userId,
      obj.sessionId,
      obj.context
    );
  }

  /**
   * Check if the query request is valid
   * @returns {boolean} True if valid, false otherwise
   */
  isValid() {
    return this.validate().length === 0;
  }
}

export default QueryRequest;