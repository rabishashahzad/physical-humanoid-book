import React from 'react';
import FloatingRagChat from '../components/RagQuery/FloatingRagChat';

// Root component that wraps the entire Docusaurus application
export default function Root({ children }) {
  return (
    <>
      {children}
      <FloatingRagChat />
    </>
  );
}