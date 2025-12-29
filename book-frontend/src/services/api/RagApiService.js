/**
 * RAG API Service - Handles communication with the backend RAG agent
 * Provides methods for querying the backend and managing API configuration
 */

// Environment-specific default configurations
const ENV_CONFIGS = {
  development: {
    backendUrl: (typeof process !== 'undefined' ? process.env.BACKEND_API_URL : null) || 'http://localhost:8000',
    timeout: 30000, // 30 seconds for development
    retries: 3,
    environment: 'development'
  },
  staging: {
    backendUrl: (typeof process !== 'undefined' ? process.env.BACKEND_API_URL : null) ||
               (typeof process !== 'undefined' ? process.env.REACT_APP_BACKEND_API_URL : null) ||
               (typeof process !== 'undefined' ? process.env.NEXT_PUBLIC_BACKEND_API_URL : null) ||
               'https://staging-api.example.com',
    timeout: 20000, // 20 seconds for staging
    retries: 2,
    environment: 'staging'
  },
  production: {
    backendUrl: (typeof process !== 'undefined' ? process.env.BACKEND_API_URL : null) ||
               (typeof process !== 'undefined' ? process.env.REACT_APP_BACKEND_API_URL : null) ||
               (typeof process !== 'undefined' ? process.env.NEXT_PUBLIC_BACKEND_API_URL : null) ||
               'https://api.example.com',
    timeout: 15000, // 15 seconds for production
    retries: 1,
    environment: 'production'
  }
};

// Get the current environment
const getCurrentEnvironment = () => {
  if (typeof window !== 'undefined') {
    // Client-side environment detection
    const hostname = window.location.hostname;
    if (hostname === 'localhost' || hostname === '127.0.0.1' || hostname.includes('local')) {
      return 'development';
    } else if (hostname.includes('staging') || hostname.includes('test')) {
      return 'staging';
    } else {
      return 'production';
    }
  } else {
    // Server-side environment detection
    return process.env.NODE_ENV || 'development';
  }
};

// Get default configuration based on environment
const getDefaultConfig = () => {
  const env = getCurrentEnvironment();
  return { ...ENV_CONFIGS[env], environment: env };
};

class RagApiService {
  constructor(config = {}) {
    // Get default config based on environment and merge with provided config
    const defaultConfig = getDefaultConfig();
    this.config = { ...defaultConfig, ...config };

    // Validate backend URL format
    try {
      new URL(this.config.backendUrl);
    } catch (error) {
      console.warn(`Invalid backend URL: ${this.config.backendUrl}. Using default for environment: ${defaultConfig.backendUrl}`);
      this.config.backendUrl = defaultConfig.backendUrl;
    }
  }

  /**
   * Update the API configuration
   * @param {Object} newConfig - New configuration values
   */
  updateConfig(newConfig) {
    this.config = { ...this.config, ...newConfig };
  }

  /**
   * Get the current API configuration
   * @returns {Object} Current configuration
   */
  getConfig() {
    return { ...this.config };
  }

  /**
   * Validate the current configuration
   * @returns {Array} Array of validation errors, empty if valid
   */
  validateConfig() {
    const errors = [];

    if (!this.config.backendUrl) {
      errors.push('backendUrl is required');
    } else {
      try {
        new URL(this.config.backendUrl);
      } catch (error) {
        errors.push(`backendUrl is not a valid URL: ${this.config.backendUrl}`);
      }
    }

    if (typeof this.config.timeout !== 'number' || this.config.timeout <= 0) {
      errors.push('timeout must be a positive number');
    }

    if (typeof this.config.retries !== 'number' || this.config.retries < 0) {
      errors.push('retries must be a non-negative number');
    }

    if (!['development', 'staging', 'production'].includes(this.config.environment)) {
      errors.push('environment must be one of: development, staging, production');
    }

    return errors;
  }

  /**
   * Make an API request with retry logic
   * @param {string} endpoint - API endpoint to call
   * @param {Object} options - Request options
   * @returns {Promise} API response
   */
  async makeRequest(endpoint, options = {}) {
    const errors = this.validateConfig();
    if (errors.length > 0) {
      throw new Error(`Configuration validation failed: ${errors.join(', ')}`);
    }

    const url = `${this.config.backendUrl}${endpoint}`;
    const maxRetries = this.config.retries;
    let lastError;

    for (let attempt = 0; attempt <= maxRetries; attempt++) {
      try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

        const response = await fetch(url, {
          ...options,
          signal: controller.signal,
          headers: {
            'Content-Type': 'application/json',
            ...options.headers,
          },
        });

        clearTimeout(timeoutId);

        // If successful, return the response
        if (response.ok) {
          return await response.json();
        }

        // If it's a client error (4xx), don't retry
        if (response.status >= 400 && response.status < 500) {
          const errorData = await response.json().catch(() => ({}));

          // Check for rate limiting (HTTP 429)
          if (response.status === 429) {
            const retryAfter = response.headers.get('Retry-After');
            const error = new Error('Rate limit exceeded. Please try again later.');
            error.response = {
              status: response.status,
              data: {
                ...errorData,
                retryAfter: retryAfter
              },
              statusText: response.statusText
            };
            error.request = true;
            throw error;
          }

          // Create an error object that matches the expected format for parsing
          const error = new Error(`Client error: ${response.status} - ${errorData.detail || response.statusText}`);
          error.response = {
            status: response.status,
            data: errorData,
            statusText: response.statusText
          };
          error.request = true; // Indicate this was a request error
          throw error;
        }

        // For server errors (5xx), continue to retry
        const errorData = await response.json().catch(() => ({}));
        const error = new Error(`Server error: ${response.status} - ${response.statusText}`);
        error.response = {
          status: response.status,
          data: errorData,
          statusText: response.statusText
        };
        error.request = true; // Indicate this was a request error
        throw error;
      } catch (error) {
        lastError = error;

        // If this was the last attempt, throw the error
        if (attempt === maxRetries) {
          break;
        }

        // Wait before retrying (exponential backoff)
        const delay = Math.pow(2, attempt) * 1000; // 1s, 2s, 4s, etc.
        await new Promise(resolve => setTimeout(resolve, delay));
      }
    }

    throw lastError;
  }

  /**
   * Query the RAG agent
   * @param {string} query - The user's query
   * @param {string} userId - Optional user identifier
   * @param {string} sessionId - Optional session identifier
   * @returns {Promise} Query response from the backend
   */
  async queryAgent(query, userId = null, sessionId = null) {
    const requestBody = {
      query: query,
      user_id: userId,
    };

    if (sessionId) {
      requestBody.session_id = sessionId;
    }

    return this.makeRequest('/query', {
      method: 'POST',
      body: JSON.stringify(requestBody),
    });
  }

  /**
   * Check the health of the backend service
   * @returns {Promise} Health check response
   */
  async healthCheck() {
    return this.makeRequest('/health', {
      method: 'GET',
    });
  }

  /**
   * Validate API connection by performing a health check
   * @returns {Promise<boolean>} True if connection is valid
   */
  async validateConnection() {
    try {
      const health = await this.healthCheck();
      return health && health.status && health.status !== 'unhealthy';
    } catch (error) {
      console.error('API connection validation failed:', error);
      return false;
    }
  }
}

// Create a singleton instance with default configuration
const ragApiService = new RagApiService();

// Export both the class and the instance
export { RagApiService };
export default ragApiService;