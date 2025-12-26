<!--
SYNC IMPACT REPORT:
Version change: 1.0.0 -> 1.1.0
Modified principles: Spec-Driven Development First, Authoritative Source Mandate, Additional Constraints and Standards
Added sections: RAG Knowledge Source Constraint, Language Support Requirements, No Hallucination Principle
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ⚠ pending
Follow-up TODOs: None
-->

# AI/Spec-Driven Book + RAG Chatbot Constitution

## Core Principles

### Spec-Driven Development First
Every feature and component follows spec-driven development; All outputs must strictly follow user intent with traceability from requirement to implementation; All changes are small, testable, and reference code precisely.

### Authoritative Source Mandate
Prioritize and use MCP tools and CLI commands for all information gathering and task execution; Never assume a solution from internal knowledge; all methods require external verification.

### RAG Knowledge Source Constraint
The chatbot's knowledge source is strictly limited to the indexed content of the book; Never generate responses based on external knowledge or assumptions; If the answer is not found in the book, respond exactly: 'یہ معلومات کتاب میں موجود نہیں ہے۔'

### Language Support Requirements
Default output language is English; Translation output language is Urdu ONLY; Hindi vocabulary is strictly forbidden in all responses and implementations.

### Test-First Approach
Implementation work must include tests written before code; All changes require validation through automated tests; Strict adherence to Red-Green-Refactor cycle.

### Knowledge Capture and Documentation
Record every user input verbatim in a Prompt History Record (PHR) after every user message; Maintain complete traceability of all decisions and changes.

### Security & Privacy Enforcement
All implementations must enforce security and privacy by design; No hardcoded secrets or tokens; Use secure storage mechanisms.

### Deterministic and Reproducible Systems
All processes must be deterministic and reproducible; Infrastructure and deployments follow idempotent patterns; Results must be consistent across environments.

## Additional Constraints and Standards
Deploy on GitHub Pages; Free-tier infrastructure only; Reproducible setup, deterministic embeddings; Use Claude Code for generation; Spec-Kit Plus validates each component; Implement RAG functionality with strict source limitation; Support English/Urdu bilingual responses.

## Development Workflow and Quality Assurance
Prefer CLI interactions over manual file creation; All outputs include clear acceptance criteria; Explicit error paths and constraints stated; Smallest viable change principle applied.

## Governance
All implementations must follow the specified architecture; Changes require validation against acceptance criteria; Code references must be provided for all modifications; Constitution supersedes ad-hoc practices.

**Version**: 1.1.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-20
