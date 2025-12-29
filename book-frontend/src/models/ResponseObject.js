/**
 * ResponseObject Model
 * Represents a response from the RAG backend
 * Based on the data-model.md specification
 */

class ResponseObject {
  /**
   * Create a new ResponseObject
   * @param {string} response - The AI-generated response text (required)
   * @param {Array} sources - List of source references used in the response (required)
   * @param {number} [confidence] - Confidence score between 0 and 1 (optional)
   * @param {string} queryId - Identifier linking to the original query (required)
   * @param {Object} [metadata] - Additional response metadata (optional)
   */
  constructor(response, sources, queryId, confidence = null, metadata = null) {
    this.response = response;
    this.sources = sources;
    this.confidence = confidence;
    this.queryId = queryId;
    this.metadata = metadata;
    this.timestamp = new Date().toISOString();
  }

  /**
   * Validate the response object
   * @returns {Array} Array of validation errors, empty if valid
   */
  validate() {
    const errors = [];

    // Validate response field
    if (!this.response) {
      errors.push('Response is required');
    } else if (typeof this.response !== 'string') {
      errors.push('Response must be a string');
    } else if (this.response.trim().length === 0) {
      errors.push('Response must not be empty');
    }

    // Validate sources field
    if (!this.sources) {
      errors.push('Sources are required');
    } else if (!Array.isArray(this.sources)) {
      errors.push('Sources must be an array');
    } else {
      // Validate each source object
      for (let i = 0; i < this.sources.length; i++) {
        const source = this.sources[i];
        if (typeof source !== 'object') {
          errors.push(`Source at index ${i} must be an object`);
          continue;
        }

        if (!source.title || typeof source.title !== 'string') {
          errors.push(`Source at index ${i} must have a valid title`);
        }

        if (!source.url || typeof source.url !== 'string') {
          errors.push(`Source at index ${i} must have a valid URL`);
        }

        if (!source.section || typeof source.section !== 'string') {
          errors.push(`Source at index ${i} must have a valid section`);
        }

        if (source.confidence !== undefined && (typeof source.confidence !== 'number' || source.confidence < 0 || source.confidence > 1)) {
          errors.push(`Source at index ${i} confidence must be a number between 0 and 1`);
        }
      }
    }

    // Validate confidence if provided
    if (this.confidence !== null && typeof this.confidence !== 'number') {
      errors.push('Confidence must be a number if provided');
    } else if (this.confidence !== null && (this.confidence < 0 || this.confidence > 1)) {
      errors.push('Confidence must be between 0 and 1 if provided');
    }

    // Validate queryId
    if (!this.queryId) {
      errors.push('queryId is required');
    } else if (typeof this.queryId !== 'string') {
      errors.push('queryId must be a string');
    }

    // Validate metadata if provided
    if (this.metadata && typeof this.metadata !== 'object') {
      errors.push('metadata must be an object if provided');
    }

    // Validate timestamp if provided
    if (this.timestamp) {
      const timestampDate = new Date(this.timestamp);
      if (isNaN(timestampDate.getTime())) {
        errors.push('timestamp must be in ISO 8601 format');
      }
    }

    return errors;
  }

  /**
   * Convert the response to a plain object for display
   * @returns {Object} Plain object representation
   */
  toObject() {
    const obj = {
      response: this.response,
      sources: this.sources,
      queryId: this.queryId,
    };

    if (this.confidence !== null) {
      obj.confidence = this.confidence;
    }

    if (this.metadata) {
      obj.metadata = this.metadata;
    }

    if (this.timestamp) {
      obj.timestamp = this.timestamp;
    }

    return obj;
  }

  /**
   * Create a ResponseObject from a plain object
   * @param {Object} obj - Plain object to convert
   * @returns {ResponseObject} New ResponseObject instance
   */
  static fromObject(obj) {
    return new ResponseObject(
      obj.response,
      obj.sources,
      obj.queryId,
      obj.confidence,
      obj.metadata
    );
  }

  /**
   * Check if the response object is valid
   * @returns {boolean} True if valid, false otherwise
   */
  isValid() {
    return this.validate().length === 0;
  }

  /**
   * Get source attribution formatted for display
   * @returns {Array} Array of formatted source objects
   */
  getFormattedSources() {
    return this.sources.map((source, index) => ({
      id: index,
      title: source.title,
      url: source.url,
      section: source.section,
      confidence: source.confidence || 0
    }));
  }
}

export default ResponseObject;