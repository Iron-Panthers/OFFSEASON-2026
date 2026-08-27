/**
 * sync-claude-skills.ts
 *
 * Keeps .opencode/skills/ in sync with .claude/commands/ automatically.
 *
 * On every opencode startup this plugin:
 *   1. Reads every *.md file in .claude/commands/
 *   2. Writes it to .opencode/skills/<name>/SKILL.md
 *
 * The .claude/commands/ files are the single source of truth.
 * .opencode/skills/ is generated — do not edit those files directly.
 *
 * Each .claude/commands/*.md file must have SKILL.md frontmatter at the top:
 *   ---
 *   name: <name>
 *   description: <one-line trigger description>
 *   ---
 */

import type { Plugin } from "@opencode-ai/plugin";
import * as fs from "node:fs";
import * as path from "node:path";

export default (async ({ worktree }) => {
  const commandsDir = path.join(worktree, ".claude", "commands");
  const skillsDir = path.join(worktree, ".opencode", "skills");

  if (!fs.existsSync(commandsDir)) return {};

  const files = fs
    .readdirSync(commandsDir)
    .filter((f) => f.endsWith(".md") && !f.startsWith("_"));

  for (const file of files) {
    const name = file.replace(/\.md$/, "");
    const content = fs.readFileSync(path.join(commandsDir, file), "utf-8");

    // Only process files that have SKILL.md frontmatter
    if (!content.startsWith("---")) continue;

    const skillDir = path.join(skillsDir, name);
    fs.mkdirSync(skillDir, { recursive: true });

    const dest = path.join(skillDir, "SKILL.md");
    const existing = fs.existsSync(dest)
      ? fs.readFileSync(dest, "utf-8")
      : null;

    // Skip write if content is already up to date (avoids unnecessary disk writes)
    if (existing !== content) {
      fs.writeFileSync(dest, content, "utf-8");
    }
  }

  // Remove any stale skill dirs that no longer have a matching command file
  if (fs.existsSync(skillsDir)) {
    const commandNames = new Set(files.map((f) => f.replace(/\.md$/, "")));
    for (const dir of fs.readdirSync(skillsDir)) {
      if (!commandNames.has(dir)) {
        fs.rmSync(path.join(skillsDir, dir), { recursive: true, force: true });
      }
    }
  }

  return {};
}) satisfies Plugin;
