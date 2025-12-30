import React from 'react';
import Layout from '@theme/Layout';
import FloatingRagChat from '../RagQuery/FloatingRagChat';

export default function LayoutWrapper(props) {
  return (
    <Layout {...props}>
      {props.children}
      <FloatingRagChat />
    </Layout>
  );
}