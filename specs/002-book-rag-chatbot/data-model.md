# Data Model: Book RAG Chatbot with Translation

**Feature**: 002-book-rag-chatbot
**Date**: 2025-12-20
**Model Version**: 1.0

## Overview

This document defines the data models for the Book RAG Chatbot with Translation feature. The system includes user management, chat functionality, book content management, and translation services, all designed to work within the constraints of isolated knowledge bases per book/module.

## Entity Models

### 1. User

**Purpose**: Represents individual users of the system with authentication and preferences

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Required | Unique identifier for the user |
| email | String(255) | Required, Unique | User's email address for authentication |
| name | String(255) | Required | User's display name |
| password_hash | String(255) | Required | Hashed password using secure algorithm |
| created_at | DateTime | Required | Account creation timestamp |
| updated_at | DateTime | Required | Last update timestamp |
| preferences | JSON | Optional | User preferences including language settings |

**Relationships**:
- One-to-Many: User → ChatSession
- Many-to-Many: User → Book (through UserBookAccess)

**Validation Rules**:
- Email must be valid email format
- Password must meet security requirements
- Name cannot be empty

### 2. Book

**Purpose**: Represents a book or module in the system with its metadata

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Required | Unique identifier for the book |
| title | String(500) | Required | Title of the book |
| author | String(255) | Required | Author of the book |
| isbn | String(13) | Optional | ISBN identifier |
| language | String(10) | Required | Primary language of the book (e.g., 'en') |
| description | Text | Optional | Book description |
| created_at | DateTime | Required | Book registration timestamp |
| updated_at | DateTime | Required | Last update timestamp |
| is_active | Boolean | Required, Default: true | Whether the book is available |

**Relationships**:
- One-to-Many: Book → BookContent
- One-to-Many: Book → KnowledgeBase
- Many-to-Many: Book → User (through UserBookAccess)

**Validation Rules**:
- Title cannot be empty
- Language must be a valid ISO language code
- ISBN must be valid format if provided

### 3. BookContent

**Purpose**: Represents chunks of content from a book used for RAG retrieval

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Required | Unique identifier for the content chunk |
| book_id | UUID | Foreign Key, Required | Reference to the book |
| content | Text | Required | The actual text content |
| chunk_index | Integer | Required | Sequential index of the chunk in the book |
| embedding_id | String(255) | Required | Reference to the Qdrant embedding ID |
| content_type | String(50) | Required | Type of content (e.g., 'paragraph', 'section') |
| page_number | Integer | Optional | Page number if applicable |
| section_title | String(255) | Optional | Title of the section |
| created_at | DateTime | Required | Creation timestamp |

**Relationships**:
- Many-to-One: BookContent → Book

**Validation Rules**:
- Content cannot be empty
- Chunk index must be non-negative
- Book reference must exist

### 4. KnowledgeBase

**Purpose**: Represents an isolated knowledge base for a specific book/module

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Required | Unique identifier for the knowledge base |
| book_id | UUID | Foreign Key, Required | Reference to the book |
| name | String(255) | Required | Name of the knowledge base |
| description | Text | Optional | Description of the knowledge base |
| qdrant_collection | String(255) | Required | Qdrant collection name for this KB |
| created_at | DateTime | Required | Creation timestamp |
| updated_at | DateTime | Required | Last update timestamp |

**Relationships**:
- Many-to-One: KnowledgeBase → Book
- One-to-Many: KnowledgeBase → ChatSession

**Validation Rules**:
- Name cannot be empty
- Book reference must exist
- Qdrant collection name must be unique

### 5. ChatSession

**Purpose**: Represents a conversation session between a user and the chatbot

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Required | Unique identifier for the chat session |
| user_id | UUID | Foreign Key, Required | Reference to the user |
| knowledge_base_id | UUID | Foreign Key, Required | Reference to the knowledge base |
| title | String(255) | Optional | Auto-generated title based on first question |
| context_mode | String(50) | Required, Default: 'entire_book' | Mode: 'entire_book' or 'selected_text_only' |
| selected_text_context | Text | Optional | Text context when in 'selected_text_only' mode |
| created_at | DateTime | Required | Session creation timestamp |
| updated_at | DateTime | Required | Last activity timestamp |
| is_active | Boolean | Required, Default: true | Whether the session is active |

**Relationships**:
- Many-to-One: ChatSession → User
- Many-to-One: ChatSession → KnowledgeBase
- One-to-Many: ChatSession → Message

**Validation Rules**:
- User and knowledge base references must exist
- Context mode must be one of the allowed values
- Selected text context only valid when mode is 'selected_text_only'

### 6. Message

**Purpose**: Represents individual messages in a chat session

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Required | Unique identifier for the message |
| chat_session_id | UUID | Foreign Key, Required | Reference to the chat session |
| sender_type | String(20) | Required | 'user' or 'assistant' |
| content | Text | Required | The message content |
| language | String(10) | Required, Default: 'en' | Language of the message ('en' or 'ur') |
| translated_content | Text | Optional | Content translated to the other language |
| retrieved_chunks | JSON | Optional | IDs of book content chunks used for response |
| created_at | DateTime | Required | Message creation timestamp |

**Relationships**:
- Many-to-One: Message → ChatSession

**Validation Rules**:
- Chat session reference must exist
- Sender type must be 'user' or 'assistant'
- Language must be a valid ISO language code
- Content cannot be empty

### 7. UserBookAccess

**Purpose**: Represents the relationship between users and books they have access to

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Required | Unique identifier for the access record |
| user_id | UUID | Foreign Key, Required | Reference to the user |
| book_id | UUID | Foreign Key, Required | Reference to the book |
| access_level | String(50) | Required, Default: 'read' | Access level ('read', 'contributor', 'admin') |
| granted_at | DateTime | Required | Timestamp when access was granted |
| granted_by | UUID | Foreign Key | User who granted access |

**Relationships**:
- Many-to-One: UserBookAccess → User (user_id)
- Many-to-One: UserBookAccess → User (granted_by)
- Many-to-One: UserBookAccess → Book

**Validation Rules**:
- User and book references must exist
- Access level must be one of the allowed values

## State Transitions

### ChatSession States
- `active`: Session is open and accepting messages
- `archived`: Session is closed but preserved for history
- `deleted`: Session is marked for deletion (soft delete)

### Message States
- `created`: Message is created but not yet processed
- `processing`: Message is being processed by RAG system
- `completed`: Message is fully processed and response generated
- `error`: Error occurred during processing

## Indexes and Performance Considerations

### Database Indexes
1. **User.email**: Unique index for authentication performance
2. **BookContent.book_id**: Index for efficient content retrieval
3. **ChatSession.user_id**: Index for user session queries
4. **ChatSession.knowledge_base_id**: Index for knowledge base queries
5. **Message.chat_session_id**: Index for session message retrieval
6. **Message.created_at**: Index for chronological message ordering

### Qdrant Considerations
- Each book's content chunks are stored in separate Qdrant collections
- Embeddings are indexed for efficient similarity search
- Metadata filtering is used to ensure proper content isolation

## Data Integrity Constraints

1. **Foreign Key Constraints**: All relationships maintain referential integrity
2. **Unique Constraints**: Prevent duplicate critical data
3. **Check Constraints**: Validate domain-specific rules (e.g., context mode values)
4. **Cascading Deletes**: Chat sessions and messages are deleted when user is deleted
5. **Soft Deletes**: Critical data is marked as deleted rather than permanently removed

## Privacy and Security Considerations

1. **Data Isolation**: Each user's chat history is isolated from other users
2. **Book Isolation**: Content from one book cannot be accessed through another book's knowledge base
3. **Audit Trail**: All access and modifications are timestamped
4. **Data Retention**: Configurable retention policies for chat history