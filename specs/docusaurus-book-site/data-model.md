# Data Model: Docusaurus Book Site with Module 1

**Feature**: Docusaurus Book Site
**Date**: 2025-12-17
**Input**: Feature specification from `/specs/docusaurus-book-site/spec.md`

## Overview

This document defines the data structures and models used in the Docusaurus-based book site, including site configuration, content organization, navigation structure, and content metadata.

## Site Configuration Model

### SiteConfig
```
SiteConfig:
  - title: string (site title)
  - tagline: string (site tagline)
  - url: string (base URL)
  - baseUrl: string (base path)
  - organizationName: string (GitHub org/user name)
  - projectName: string (repository name)
  - deploymentBranch: string (branch for deployment)
  - trailingSlash: boolean (whether to add trailing slashes)
  - onBrokenLinks: string (behavior for broken links)
  - onBrokenMarkdownLinks: string (behavior for broken MD links)
  - favicon: string (path to favicon)
  - presets: array<PresetConfig>
  - themes: array<ThemeConfig>
  - plugins: array<PluginConfig>
  - themeConfig: ThemeConfig
```

### PresetConfig
```
PresetConfig:
  - name: string (e.g., "@docusaurus/preset-classic")
  - options: PresetOptions
```

### PresetOptions
```
PresetOptions:
  - docs: DocsOptions
  - blog: BlogOptions (optional)
  - theme: ThemeOptions
  - sitemap: SitemapOptions (optional)
  - gtag: GtagOptions (optional)
```

## Content Organization Model

### Document
```
Document:
  - id: string (unique identifier)
  - title: string (page title)
  - description: string (page description)
  - slug: string (URL path)
  - sidebar_position: number (position in sidebar)
  - tags: array<string> (content tags)
  - frontMatter: FrontMatter (metadata)
  - content: string (Markdown content)
  - source: string (file path)
```

### FrontMatter
```
FrontMatter:
  - title: string (page title)
  - description: string (page description)
  - sidebar_label: string (label in sidebar)
  - sidebar_position: number (position in sidebar)
  - hide_table_of_contents: boolean (hide TOC)
  - keywords: array<string> (SEO keywords)
  - image: string (social card image)
```

## Navigation Structure Model

### Sidebar
```
Sidebar:
  - label: string (sidebar name)
  - items: array<SidebarItem>
  - collapsible: boolean (can collapse)
  - collapsed: boolean (default collapsed state)
```

### SidebarItem
```
SidebarItem:
  - type: SidebarItemType (doc, link, category, html)
  - label: string (display text)
  - id: string (document ID, for type=doc)
  - href: string (URL, for type=link)
  - items: array<SidebarItem> (sub-items, for type=category)
  - className: string (CSS class)
  - customProps: object (additional properties)
```

### SidebarItemType
```
SidebarItemType:
  - doc: Reference to a document
  - link: External or internal link
  - category: Group of items
  - html: HTML content
```

## Module 1 Content Model

### Module
```
Module:
  - id: string (module identifier)
  - title: string (module title)
  - description: string (module description)
  - chapters: array<Chapter>
  - position: number (order in book)
  - sidebar_position: number (position in sidebar)
```

### Chapter
```
Chapter:
  - id: string (chapter identifier)
  - title: string (chapter title)
  - description: string (chapter description)
  - content: string (Markdown content)
  - prerequisites: array<string> (required knowledge)
  - objectives: array<string> (learning objectives)
  - exercises: array<Exercise> (practice problems)
  - position: number (order in module)
  - sidebar_position: number (position in sidebar)
```

### Exercise
```
Exercise:
  - id: string (exercise identifier)
  - title: string (exercise title)
  - description: string (exercise description)
  - difficulty: string (easy/medium/hard)
  - type: string (coding, theory, practical)
  - solution: string (solution content)
  - hints: array<string> (helpful hints)
```

## Docusaurus-Specific Models

### DocsOptions
```
DocsOptions:
  - sidebarPath: string (path to sidebar config)
  - editUrl: string (URL for editing)
  - showLastUpdateTime: boolean (show last update)
  - showLastUpdateAuthor: boolean (show author)
  - remarkPlugins: array<Plugin> (Markdown plugins)
  - rehypePlugins: array<Plugin> (HTML plugins)
  - beforeDefaultRemarkPlugins: array<Plugin>
  - beforeDefaultRehypePlugins: array<Plugin>
```

### ThemeConfig
```
ThemeConfig:
  - navbar: NavbarConfig
  - footer: FooterConfig
  - prism: PrismConfig
  - colorMode: ColorModeConfig
```

### NavbarConfig
```
NavbarConfig:
  - title: string (navbar title)
  - logo: LogoConfig (optional)
  - items: array<NavbarItem>
```

### NavbarItem
```
NavbarItem:
  - type: string (doc, docsVersionDropdown, etc.)
  - position: string (left, right)
  - label: string (display text)
  - to: string (internal link)
  - href: string (external link)
```

### MDX Content Structure
```
MDXContent:
  - imports: array<ImportStatement> (React component imports)
  - frontmatter: FrontMatter (metadata)
  - body: array<ContentNode> (content structure)
```

### ContentNode
```
ContentNode:
  - type: ContentType (text, code, heading, list, jsx, etc.)
  - content: string (node content)
  - props: object (node properties)
  - children: array<ContentNode> (nested content)
```

## Example Module 1 Structure

```
Book:
  - title: "Robotic Nervous System: A Guide to ROS 2"
  - modules:
    - Module:
      - id: "module-1-robotic-nervous-system"
      - title: "Module 1: Robotic Nervous System (ROS 2)"
      - description: "Learn ROS 2 middleware for humanoid control, Python agents integration, and URDF modeling"
      - chapters:
        - Chapter:
          - id: "chapter-1-ros2-basics"
          - title: "Chapter 1: ROS 2 Basics"
          - description: "Understanding nodes, topics, and services"
          - position: 1
          - sidebar_position: 1
        - Chapter:
          - id: "chapter-2-python-ros2"
          - title: "Chapter 2: Python & ROS 2"
          - description: "Using rclpy, publishers, and subscribers"
          - position: 2
          - sidebar_position: 2
        - Chapter:
          - id: "chapter-3-urdf-modeling"
          - title: "Chapter 3: URDF Modeling"
          - description: "Links, joints, sensors, and simple humanoid"
          - position: 3
          - sidebar_position: 3
```

## Deployment Configuration

### DeploymentConfig
```
DeploymentConfig:
  - provider: string (github, netlify, etc.)
  - organizationName: string (GitHub org/user name)
  - projectName: string (repository name)
  - deploymentBranch: string (e.g., gh-pages)
  - buildCommand: string (command to build site)
  - outputDirectory: string (build output path)
  - environment: object (deployment environment vars)
```