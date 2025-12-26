import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';

export default function Button({
  children,
  href,
  variant = 'primary',
  size = 'md',
  block = false,
  disabled = false,
  icon,
  className,
  ...props
}) {
  const buttonClasses = clsx(
    'button-custom',
    `button--${variant}`,
    `button--${size}`,
    {
      'button--block': block,
      'button--disabled': disabled,
      'button--with-icon': icon,
    },
    className
  );

  const content = (
    <>
      {icon && <span className="button__icon">{icon}</span>}
      <span className="button__text">{children}</span>
    </>
  );

  if (href) {
    return (
      <Link to={href} className={buttonClasses} {...props}>
        {content}
      </Link>
    );
  }

  return (
    <button className={buttonClasses} disabled={disabled} {...props}>
      {content}
    </button>
  );
}

export const buttonStyles = `
  .button-custom {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: var(--ifm-spacing-2);
    border: none;
    border-radius: var(--ifm-spacing-2);
    font-weight: var(--ifm-font-weight-semibold);
    text-decoration: none;
    cursor: pointer;
    transition: var(--ifm-transition-all);
    position: relative;
    overflow: hidden;
    min-height: 44px;
    padding: var(--ifm-spacing-2) var(--ifm-spacing-4);
    font-size: var(--ifm-font-size-base);
    line-height: var(--ifm-line-height-base);
  }

  .button-custom:hover {
    text-decoration: none;
    transform: translateY(-2px);
    box-shadow: var(--ifm-shadow-md);
  }

  .button-custom:focus {
    outline: 2px solid var(--ifm-color-primary);
    outline-offset: 2px;
  }

  /* Variants */
  .button--primary {
    background-color: var(--ifm-color-primary);
    color: var(--ifm-color-text-inverse);
  }

  .button--primary:hover:not(:disabled) {
    background-color: var(--ifm-color-primary-dark);
  }

  .button--secondary {
    background-color: var(--ifm-color-secondary);
    color: var(--ifm-color-text-inverse);
  }

  .button--secondary:hover:not(:disabled) {
    background-color: var(--ifm-color-secondary-dark);
  }

  .button--success {
    background-color: var(--ifm-color-success);
    color: var(--ifm-color-text-inverse);
  }

  .button--success:hover:not(:disabled) {
    background-color: var(--ifm-color-success-dark);
  }

  .button--outline-primary {
    background-color: transparent;
    color: var(--ifm-color-primary);
    border: 2px solid var(--ifm-color-primary);
  }

  .button--outline-primary:hover:not(:disabled) {
    background-color: var(--ifm-color-primary);
    color: var(--ifm-color-text-inverse);
  }

  /* Sizes */
  .button--sm {
    padding: var(--ifm-spacing-1) var(--ifm-spacing-3);
    font-size: var(--ifm-font-size-sm);
    min-height: 36px;
  }

  .button--lg {
    padding: var(--ifm-spacing-3) var(--ifm-spacing-6);
    font-size: var(--ifm-font-size-lg);
    min-height: 52px;
  }

  .button--xl {
    padding: var(--ifm-spacing-4) var(--ifm-spacing-8);
    font-size: var(--ifm-font-size-xl);
    min-height: 60px;
  }

  /* Block */
  .button--block {
    display: flex;
    width: 100%;
  }

  /* Disabled */
  .button--disabled {
    opacity: 0.6;
    cursor: not-allowed;
  }

  .button--disabled:hover {
    transform: none;
    box-shadow: none;
  }

  /* With icon */
  .button--with-icon {
    padding: var(--ifm-spacing-2) var(--ifm-spacing-5);
  }

  .button__icon {
    display: flex;
    align-items: center;
  }

  .button__text {
    display: flex;
    align-items: center;
  }
`;