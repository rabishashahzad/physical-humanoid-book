# Implementation Plan: Backend-Frontend Integration

**Branch**: `004-backend-frontend-integration` | **Date**: 2025-12-29 | **Spec**: specs/004-backend-frontend-integration/spec.md
**Input**: Feature specification from `/specs/004-backend-frontend-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Integrate the RAG backend (FastAPI service from Spec 3) with the Docusaurus frontend by establishing a local and production-ready connection to the FastAPI service. This involves configuring FastAPI endpoint URLs via frontend environment variables, adding frontend client logic to call the RAG API, sending user queries from the book UI to the backend, receiving and rendering agent responses in the frontend, and validating end-to-end flow in local and deployed environments.

## Technical Context

**Language/Version**: JavaScript/TypeScript for frontend, Python 3.11 for backend
**Primary Dependencies**: Docusaurus framework for frontend, FastAPI for backend, HTTP client libraries (axios/fetch)
**Storage**: N/A (no new storage - uses existing backend Qdrant and frontend static files)
**Testing**: Jest for frontend unit tests, pytest for backend integration tests
**Target Platform**: Web browser (frontend), Linux server (backend)
**Project Type**: Web (frontend + backend)
**Performance Goals**: <10 second response time for 95% of queries, <500ms API call latency
**Constraints**: Must work in local development and deployed environments, CORS configuration for cross-domain communication, environment-based API URL configuration
**Scale/Scope**: Single Docusaurus site connecting to single RAG backend service

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Check:
- ✅ Spec-Driven Development First: Following spec from `/specs/004-backend-frontend-integration/spec.md`
- ✅ Authoritative Source Mandate: Using MCP tools and CLI commands for verification
- ✅ RAG Knowledge Source Constraint: Integration will maintain source limitations from existing backend
- ✅ Language Support Requirements: Frontend will support English/Urdu bilingual responses as per backend
- ✅ Test-First Approach: Will implement tests for frontend API integration
- ✅ Knowledge Capture and Documentation: Creating PHR for this planning session
- ✅ Security & Privacy Enforcement: Using environment variables for API configuration
- ✅ Deterministic and Reproducible Systems: Configuration via environment variables for consistency
- ✅ Additional Constraints: Using free-tier infrastructure, deployable on GitHub Pages

## Project Structure

### Documentation (this feature)

```text
specs/004-backend-frontend-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

book-frontend/
├── src/
│   ├── components/
│   │   └── RagQuery/
│   │       ├── RagQueryComponent.jsx    # Main RAG query component
│   │       ├── RagQueryService.js       # Frontend API client
│   │       └── RagQueryStyles.css       # Component styling
│   ├── pages/
│   └── services/
│       └── api/
│           └── RagApiService.js         # Backend API integration
└── docusaurus.config.js                 # Configuration for API endpoints

backend/
└── src/
    └── api/
        └── agent.py                     # FastAPI RAG agent (from Spec 3)

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [None]    | [None]     | [No violations identified]          |
