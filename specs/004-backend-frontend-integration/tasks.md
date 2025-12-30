# Tasks: Backend-Frontend Integration

**Feature**: 004-backend-frontend-integration | **Date**: 2025-12-29 | **Spec**: specs/004-backend-frontend-integration/spec.md
**Input**: Feature spec from `/specs/004-backend-frontend-integration/spec.md`, Implementation plan from `/specs/004-backend-frontend-integration/plan.md`

## Summary

Implementation of backend-frontend integration to connect the Docusaurus frontend with the FastAPI RAG backend. This enables users to submit queries from the frontend UI and receive AI-generated responses from the RAG agent with proper source attribution.

## Implementation Strategy

- **MVP Scope**: Implement User Story 1 (P1) first - basic query functionality with response display
- **Incremental Delivery**: Complete each user story as a standalone, testable increment
- **Parallel Opportunities**: API client development, component creation, and testing can run in parallel across different files
- **Testing Approach**: Jest for frontend unit tests, pytest for backend integration tests

## Dependencies

- User Story 2 (P1) must be completed before User Story 1 (P1) can be fully functional
- User Story 3 (P2) depends on both User Story 1 and User Story 2

## Parallel Execution Examples

- **Per Story**: Component development, API service implementation, and styling can run in parallel
- **Across Stories**: Error handling components can be developed while query functionality is being implemented

---

## Phase 1: Setup

**Goal**: Establish project structure and dependencies for the integration

- [X] T001 Set up frontend API service directory structure at `book-frontend/src/services/api/`
- [X] T002 Set up frontend components directory structure at `book-frontend/src/components/RagQuery/`
- [X] T003 Install required frontend dependencies (axios or fetch wrapper) for API communication
- [X] T004 Verify backend API endpoints are available and accessible for integration

## Phase 2: Foundational

**Goal**: Create shared infrastructure components that all user stories depend on

- [X] T005 [P] Create frontend API configuration service to handle environment variables in `book-frontend/src/services/api/RagApiService.js`
- [X] T006 [P] Implement environment-based API URL configuration in `book-frontend/docusaurus.config.js`
- [X] T007 [P] Create API request/response models based on data-model.md in `book-frontend/src/models/QueryRequest.js`
- [X] T008 [P] Create API request/response models based on data-model.md in `book-frontend/src/models/ResponseObject.js`
- [X] T009 [P] Set up CORS configuration in FastAPI backend to allow frontend domain requests

## Phase 3: [US1] Query the RAG Agent from Docusaurus Frontend (Priority: P1)

**Goal**: Enable users to submit questions and receive AI-generated responses with source attribution

**Independent Test**: Submit a query through the frontend UI component and verify that the response comes from the backend RAG agent with proper source attribution

**Tests** (if requested):
- [ ] T010 [P] [US1] Create unit tests for RagQueryComponent.jsx functionality
- [ ] T011 [P] [US1] Create integration tests for frontend-backend API communication

**Implementation**:
- [X] T012 [P] [US1] Create RagQueryComponent.jsx with query input field and submission handling
- [X] T013 [P] [US1] Implement query validation logic (1-1000 characters) in RagQueryComponent.jsx
- [X] T014 [US1] Create RagQueryService.js to handle HTTP communication with backend API
- [X] T015 [P] [US1] Implement POST /query API call in RagQueryService.js following API contract
- [X] T016 [P] [US1] Create response display component in RagQueryComponent.jsx with source attribution
- [X] T017 [P] [US1] Implement response parsing and formatting from backend API in RagQueryComponent.jsx
- [X] T018 [US1] Add proper source attribution display with clickable links to book sections
- [X] T019 [US1] Integrate RagQueryComponent into Docusaurus layout/pages as specified in plan.md

## Phase 4: [US2] Configure Backend API Connection (Priority: P1)

**Goal**: Allow developers to configure API endpoints via environment variables without code changes

**Independent Test**: Configure different API endpoints and verify that the frontend successfully communicates with each backend environment

**Tests** (if requested):
- [ ] T020 [P] [US2] Create unit tests for API configuration service with different environments

**Implementation**:
- [X] T021 [P] [US2] Enhance RagApiService.js to support multiple environment configurations
- [X] T022 [P] [US2] Implement environment detection logic (development, staging, production) in RagApiService.js
- [X] T023 [US2] Update docusaurus.config.js to properly read and expose environment variables
- [X] T024 [P] [US2] Add timeout and retry configuration options to API service based on data-model.md
- [X] T025 [P] [US2] Create API connection validation function in RagApiService.js
- [X] T026 [US2] Implement API endpoint switching mechanism based on environment variables
- [X] T027 [US2] Add configuration validation for backendUrl format and accessibility

## Phase 5: [US3] Handle API Errors Gracefully (Priority: P2)

**Goal**: Provide meaningful error messages during backend failures to improve user experience

**Independent Test**: Simulate backend failures and verify that appropriate error messages are displayed

**Tests** (if requested):
- [ ] T028 [P] [US3] Create unit tests for error handling scenarios in RagQueryComponent.jsx

**Implementation**:
- [X] T029 [P] [US3] Implement error response parsing in RagQueryService.js following API contract
- [X] T030 [P] [US3] Create error display component in RagQueryComponent.jsx for different error types
- [X] T031 [P] [US3] Add network error handling (timeout, connection failure) in RagQueryService.js
- [X] T032 [US3] Implement proper error messages for QUERY_TOO_LONG, QUERY_EMPTY, INVALID_FORMAT codes
- [X] T033 [P] [US3] Add backend unavailability error handling (BACKEND_UNAVAILABLE, PROCESSING_ERROR)
- [X] T034 [P] [US3] Create user-friendly error messages for different failure scenarios
- [X] T035 [US3] Add retry logic for transient failures following data-model.md business rules
- [X] T036 [US3] Implement timeout handling with appropriate user feedback in RagQueryComponent.jsx

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Enhance the implementation with additional features and optimizations

- [X] T037 [P] Add input validation for query fields before sending to backend (FR-007)
- [X] T038 [P] Implement proper loading states and user feedback during query processing
- [X] T039 Add response metadata display (confidence indicators, processing time) as per FR-008
- [X] T040 [P] Add proper accessibility attributes to RagQueryComponent.jsx
- [X] T041 [P] Implement rate limiting handling and user notifications
- [X] T042 [P] Add analytics/tracking for query usage (optional based on spec constraints)
- [X] T043 [P] Create comprehensive end-to-end tests for the complete query flow
- [X] T044 [P] Add proper documentation and comments to all new components and services
- [X] T045 [P] Perform final testing across different environments (development, staging, production)
- [X] T046 [P] Optimize component performance and bundle size for production deployment