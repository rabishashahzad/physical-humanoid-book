---
description: "Task list for book embeddings ingestion pipeline"
---

# Tasks: Book Embeddings and Vector Storage

**Input**: Design documents from `/specs/[001-book-embeddings]/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification indicates tests should be included for each component.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- **Paths shown below assume the web app structure from plan.md**

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create backend project structure per implementation plan in backend/
- [x] T002 Initialize Python 3.11 project with dependencies in backend/requirements.txt
- [x] T003 [P] Create .env.example with required environment variables in backend/.env.example
- [x] T004 [P] Configure pytest testing framework in backend/
- [x] T005 Create project configuration structure in backend/src/lib/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Create configuration management module in backend/src/lib/config.py
- [x] T007 Create base data models for the application in backend/src/models/embedding.py
- [x] T008 Setup error handling and logging infrastructure in backend/src/lib/
- [x] T009 Create CLI entry point structure in backend/src/cli/ingestion_pipeline.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Deploy Book URLs and Extract Content (Priority: P1) 🎯 MVP

**Goal**: Implement a web crawler that can discover and extract clean text content from deployed Docusaurus book URLs

**Independent Test**: Can be fully tested by running the crawler against the deployed Vercel URLs and verifying that text content is extracted without errors, delivering the raw data needed for the embedding pipeline.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [x] T010 [P] [US1] Create unit tests for crawler functionality in backend/tests/unit/test_crawler.py
- [x] T011 [P] [US1] Create integration tests for content extraction in backend/tests/integration/test_crawler.py

### Implementation for User Story 1

- [x] T012 [P] [US1] Implement URL discovery and crawling service in backend/src/services/crawler.py
- [x] T013 [US1] Implement content extraction from HTML pages in backend/src/services/crawler.py (depends on T012)
- [x] T014 [US1] Implement text cleaning and normalization in backend/src/services/text_processor.py
- [x] T015 [US1] Implement text chunking with appropriate size and overlap in backend/src/services/text_processor.py
- [x] T016 [US1] Add error handling for inaccessible URLs in backend/src/services/crawler.py
- [x] T017 [US1] Add validation to ensure content extraction preserves semantic meaning
- [x] T018 [US1] Implement CLI command for URL discovery and content extraction in backend/src/cli/ingestion_pipeline.py

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Generate Embeddings from Extracted Content (Priority: P1)

**Goal**: Convert extracted text content into vector embeddings using Cohere models for semantic search capabilities

**Independent Test**: Can be fully tested by taking extracted text chunks and generating embeddings, then verifying the embeddings have the expected format and dimensionality.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [x] T019 [P] [US2] Create unit tests for embedding generation in backend/tests/unit/test_embedding_generator.py
- [x] T020 [P] [US2] Create integration tests for Cohere API integration in backend/tests/integration/test_embedding_generator.py

### Implementation for User Story 2

- [x] T021 [P] [US2] Implement Cohere API integration for embedding generation in backend/src/services/embedding_generator.py
- [x] T022 [US2] Add batch processing for efficient embedding generation in backend/src/services/embedding_generator.py (depends on T021)
- [x] T023 [US2] Implement retry logic with exponential backoff for API calls in backend/src/services/embedding_generator.py
- [x] T024 [US2] Add validation to ensure embeddings have consistent dimensions in backend/src/services/embedding_generator.py
- [x] T025 [US2] Implement rate limit handling for Cohere API in backend/src/services/embedding_generator.py
- [x] T026 [US2] Add timing and performance metrics for embedding generation in backend/src/services/embedding_generator.py
- [x] T027 [US2] Integrate embedding generation with text chunking from US1 in backend/src/cli/ingestion_pipeline.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Store Embeddings in Vector Database (Priority: P2)

**Goal**: Store generated embeddings in Qdrant Cloud for efficient querying in future RAG applications

**Independent Test**: Can be fully tested by storing embeddings in Qdrant and verifying they can be retrieved by ID, delivering persistent storage capability.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [x] T028 [P] [US3] Create unit tests for Qdrant storage operations in backend/tests/unit/test_vector_storage.py
- [x] T029 [P] [US3] Create integration tests for vector storage and retrieval in backend/tests/integration/test_vector_storage.py

### Implementation for User Story 3

- [x] T030 [P] [US3] Implement Qdrant Cloud integration in backend/src/services/vector_storage.py
- [x] T031 [US3] Create Qdrant collection and index setup in backend/src/services/vector_storage.py (depends on T030)
- [x] T032 [US3] Implement embedding storage with metadata in backend/src/services/vector_storage.py
- [x] T033 [US3] Implement embedding retrieval functionality in backend/src/services/vector_storage.py
- [x] T034 [US3] Add proper indexing for efficient vector search operations in backend/src/services/vector_storage.py
- [x] T035 [US3] Add error handling for Qdrant Cloud unavailability in backend/src/services/vector_storage.py
- [x] T036 [US3] Implement basic vector search functionality for testing in backend/src/services/vector_storage.py
- [x] T037 [US3] Integrate vector storage with embedding generation from US2 in backend/src/cli/ingestion_pipeline.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T038 [P] Update documentation in backend/README.md with usage instructions
- [x] T039 Code cleanup and refactoring across all services
- [x] T040 Performance optimization for memory usage under 100MB constraint
- [x] T041 [P] Additional unit tests for edge cases in backend/tests/unit/
- [x] T042 Security hardening for API key handling
- [x] T043 Run quickstart.md validation and update if needed
- [x] T044 Implement comprehensive logging across all components
- [x] T045 Add progress tracking and reporting to CLI interface
- [x] T046 Create smoke test for end-to-end pipeline validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Create unit tests for crawler functionality in backend/tests/unit/test_crawler.py"
Task: "Create integration tests for content extraction in backend/tests/integration/test_crawler.py"

# Launch all models for User Story 1 together:
Task: "Implement URL discovery and crawling service in backend/src/services/crawler.py"
Task: "Implement content extraction from HTML pages in backend/src/services/crawler.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence