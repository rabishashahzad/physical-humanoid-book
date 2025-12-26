import React, { useState, useEffect } from 'react';
import Navbar from '@theme-original/Navbar';
import clsx from 'clsx';

// Custom Navbar component with enhanced functionality
export default function CustomNavbar(props) {
  const [scrolled, setScrolled] = useState(false);

  useEffect(() => {
    const handleScroll = () => {
      setScrolled(window.scrollY > 10);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <header
      className={clsx('navbar-enhanced', {
        'navbar-scrolled': scrolled,
      })}
    >
      <Navbar {...props} />
      <style jsx>{`
        .navbar-enhanced {
          transition: all 0.3s ease;
          box-shadow: 0 2px 4px rgba(0, 0, 0, 0.05);
        }

        .navbar-scrolled {
          box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06);
          backdrop-filter: blur(10px);
          background-color: rgba(255, 255, 255, 0.95);
        }
      `}</style>
    </header>
  );
}