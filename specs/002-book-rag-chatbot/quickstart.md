# Quickstart Guide: Book RAG Chatbot with Translation

**Feature**: 002-book-rag-chatbot
**Date**: 2025-12-20

## Overview

This guide provides a quick start for developers to set up and run the Book RAG Chatbot with Translation system. The system consists of a React-based frontend and a FastAPI backend with Qdrant vector storage and Neon Postgres database.

## Prerequisites

- Python 3.11+
- Node.js 18+ and npm/yarn
- Docker (for local development)
- Git
- An API key for OpenAI (or alternative LLM service)

## Environment Setup

### Backend (FastAPI)

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Set up Python virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install Python dependencies**:
   ```bash
   pip install -r backend/requirements.txt
   ```

4. **Set up environment variables**:
   Create a `.env` file in the backend directory with the following:
   ```env
   DATABASE_URL=postgresql://username:password@localhost:5432/book_rag_chatbot
   QDRANT_URL=https://your-qdrant-cluster.qdrant.tech
   QDRANT_API_KEY=your-qdrant-api-key
   OPENAI_API_KEY=your-openai-api-key
   SECRET_KEY=your-secret-key-for-jwt
   BETTER_AUTH_SECRET=your-better-auth-secret
   ```

5. **Run database migrations**:
   ```bash
   cd backend
   alembic upgrade head
   ```

6. **Start the backend server**:
   ```bash
   uvicorn src.main:app --reload --port 8000
   ```

### Frontend (React)

1. **Navigate to frontend directory**:
   ```bash
   cd frontend
   ```

2. **Install dependencies**:
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Set up environment variables**:
   Create a `.env` file in the frontend directory:
   ```env
   REACT_APP_API_URL=http://localhost:8000
   REACT_APP_TRANSLATION_ENABLED=true
   ```

4. **Start the development server**:
   ```bash
   npm start
   # or
   yarn start
   ```

## Configuration

### Qdrant Setup

1. Sign up for Qdrant Cloud (free tier) at [qdrant.tech](https://qdrant.tech)
2. Create a new cluster
3. Note the URL and API key
4. Update your environment variables

### Neon Postgres Setup

1. Create a Neon project (free tier)
2. Create a database for the application
3. Note the connection string
4. Update your environment variables

### BetterAuth Configuration

1. The system uses BetterAuth for authentication
2. Configure your OAuth providers if needed
3. Set the appropriate environment variables

## Running Tests

### Backend Tests
```bash
cd backend
pytest
```

### Frontend Tests
```bash
cd frontend
npm test
# or
yarn test
```

## Key Endpoints

### Authentication
- `POST /auth/register` - User registration
- `POST /auth/login` - User login
- `POST /auth/logout` - User logout

### Books
- `GET /books` - List available books
- `GET /books/{bookId}` - Get book details

### Chat
- `POST /chat/{bookId}/sessions` - Create a new chat session
- `GET /chat/sessions/{sessionId}/messages` - Get messages in a session
- `POST /chat/sessions/{sessionId}/query` - Submit a query to RAG system

### Translation
- `POST /translate` - Translate text between English and Urdu

## Development Workflow

1. **Feature Development**:
   - Create a new branch: `git checkout -b feature/your-feature-name`
   - Implement your changes
   - Write tests
   - Run tests: `pytest` (backend) and `npm test` (frontend)
   - Commit and push: `git push origin feature/your-feature-name`

2. **Database Changes**:
   - Make changes to models in `backend/src/models/`
   - Generate migration: `alembic revision --autogenerate -m "description of changes"`
   - Apply migration: `alembic upgrade head`

3. **API Contract Changes**:
   - Update the OpenAPI spec in `specs/002-book-rag-chatbot/contracts/openapi.yaml`
   - Regenerate client/server code if needed

## Troubleshooting

### Common Issues

1. **Database Connection Errors**:
   - Verify your Neon Postgres connection string
   - Check that the database is running and accessible

2. **Qdrant Connection Errors**:
   - Verify your Qdrant URL and API key
   - Check that your account is active and within limits

3. **Authentication Issues**:
   - Verify your BetterAuth configuration
   - Check that JWT secret is properly set

4. **Translation Not Working**:
   - Verify your Hugging Face or translation service API keys
   - Check that the translation models are properly loaded

### Useful Commands

- **Check system status**: `curl http://localhost:8000/health`
- **View API documentation**: `http://localhost:8000/docs` (Swagger UI)
- **View alternative docs**: `http://localhost:8000/redoc` (ReDoc)

## Next Steps

1. Add your book content to the system
2. Configure the vector embeddings for your book
3. Test the RAG functionality with sample questions
4. Customize the UI to match your requirements