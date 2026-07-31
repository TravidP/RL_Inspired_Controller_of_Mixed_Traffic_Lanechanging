import fs from "node:fs/promises";
import path from "node:path";
import { FileBlob, PresentationFile } from "@oai/artifact-tool";

const ROOT = "/Users/shuwei/github project/RL_Inspired_Controller_of_Mixed_Traffic_Lanechanging";
const TMP = path.join(ROOT, ".codex-tmp/ifac-editable");
const OUT = path.join(ROOT, "Docs/IFAC2026_PARC_DualFormat/IFAC2026_PARC_Methodology_Editable.pptx");
const SIMPLE = "/Users/shuwei/.codex/plugins/cache/openai-curated-remote/openai-templates/0.1.0/skills/artifact-template-simple-light-mode/assets/reference.pptx";
const OLD_DECK = path.join(ROOT, "Docs/IFAC2026_PARC_DualFormat/IFAC2026_PARC_Methodology.pptx");

const W = 1280;
const H = 720;
const RED = "#CC0000";
const DARK = "#4C4C4C";
const TEXT = "#111111";
const MUTED = "#666666";
const LIGHT = "#F3F3F3";
const LINE = "#D9D9D9";
const BLUE = "#1167A8";
const GREEN = "#2E8B57";

const assets = {
  logo: path.join(ROOT, "Docs/IFAC2026_PARC_DualFormat/assets/uog-logo.png"),
  allHuman: path.join(ROOT, "Docs/trajectory_log_allhuman44.png"),
  ring: path.join(ROOT, "Docs/doublelaneringnetwork_2.png"),
  oneAv: path.join(ROOT, "Docs/trajectory_log_1AVLC.png"),
  overview: path.join(ROOT, "Docs/The overview of paired controller for stabilizing flow.png"),
  paired: path.join(ROOT, "Docs/trajectory_log_paired_controller.png"),
  speed: path.join(ROOT, "Docs/avg_speed_comparison.png"),
  ad: path.join(TMP, "ifac29-page1.png"),
};

async function bytes(file) {
  const data = await fs.readFile(file);
  return data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength);
}

function addShape(slide, geometry, x, y, w, h, fill = "none", lineFill = "none", lineWidth = 0, name) {
  return slide.shapes.add({
    geometry,
    name,
    position: { left: x, top: y, width: w, height: h },
    fill,
    line: { style: "solid", fill: lineFill, width: lineWidth },
  });
}

function addText(slide, value, x, y, w, h, options = {}) {
  const box = addShape(slide, "textbox", x, y, w, h, options.fill ?? "none", options.lineFill ?? "none", options.lineWidth ?? 0, options.name);
  if (Array.isArray(value) || (value && typeof value === "object")) box.text.set(value);
  else box.text = value;
  box.text.style = {
    fontSize: options.fontSize ?? 18,
    bold: options.bold ?? false,
    color: options.color ?? TEXT,
    alignment: options.alignment ?? "left",
    verticalAlignment: options.verticalAlignment ?? "top",
    typeface: options.typeface ?? "Arial",
    lineSpacing: options.lineSpacing ?? 1.05,
    autoFit: options.autoFit ?? "shrinkText",
    insets: options.insets ?? { top: 3, right: 4, bottom: 3, left: 4 },
  };
  return box;
}

function bulletParagraphs(items, color = TEXT) {
  return items.map((item) => ({
    bulletCharacter: "•",
    marginLeft: 22,
    indent: -13,
    spaceAfter: 8,
    runs: [{ run: item, textStyle: { color } }],
  }));
}

function numberedParagraphs(items) {
  return items.map((item, i) => ({
    runs: [
      { run: `${i + 1}.  `, textStyle: { color: RED, bold: true } },
      { run: item, textStyle: { color: TEXT } },
    ],
    spaceAfter: 10,
  }));
}

async function addImage(slide, file, x, y, w, h, alt, fit = "contain") {
  return slide.images.add({
    blob: await bytes(file),
    contentType: "image/png",
    alt,
    fit,
    position: { left: x, top: y, width: w, height: h },
  });
}

async function addChrome(slide, title, number, takeaway, source, logoBytes) {
  slide.background.fill = "#FFFFFF";
  slide.images.add({ blob: logoBytes, contentType: "image/png", alt: "University of Groningen logo", fit: "contain", position: { left: 28, top: 14, width: 145, height: 40 } });
  addShape(slide, "rect", 0, 56, W, 5, RED, RED, 0, "UoG red rule");
  addText(slide, String(number), 1182, 20, 55, 24, { fontSize: 12, color: RED, alignment: "right", verticalAlignment: "middle", name: "Slide number" });
  const titleSize = title.length > 68 ? 27 : title.length > 58 ? 29 : 31;
  addText(slide, title, 48, 72, 1178, 60, { fontSize: titleSize, bold: true, color: TEXT, lineSpacing: 0.95, name: "Slide title" });
  if (takeaway) {
    addShape(slide, "rect", 48, 615, 1184, 38, DARK, DARK, 0, "Takeaway band");
    addText(slide, takeaway, 62, 620, 1156, 28, { fontSize: 15, bold: true, color: "#FFFFFF", alignment: "center", verticalAlignment: "middle", name: "Takeaway" });
  }
  addText(slide, `Source: ${source}`, 648, 666, 575, 20, { fontSize: 8.5, color: MUTED, alignment: "right", verticalAlignment: "middle", name: "Source footer" });
}

function addNativeTable(slide, x, y, widths, rowHeights, values, options = {}) {
  let yy = y;
  for (let r = 0; r < values.length; r += 1) {
    let xx = x;
    for (let c = 0; c < values[r].length; c += 1) {
      const header = r === 0;
      addShape(slide, "rect", xx, yy, widths[c], rowHeights[r], header ? (options.headerFill ?? "#FFFFFF") : (r % 2 === 0 ? "#FAFAFA" : "#FFFFFF"), LINE, 1, `Table r${r}c${c}`);
      addText(slide, values[r][c], xx + 7, yy + 5, widths[c] - 14, rowHeights[r] - 10, {
        fontSize: header ? 15 : 14,
        bold: header || (options.boldFirstColumn && c === 0),
        color: header ? RED : TEXT,
        verticalAlignment: "middle",
        lineSpacing: 0.95,
      });
      xx += widths[c];
    }
    yy += rowHeights[r];
  }
}

function clearLocalElements(slide) {
  for (const element of [...slide.elements.items]) slide.elements.deleteById(element.id);
}

async function main() {
  await fs.mkdir(TMP, { recursive: true });
  const presentation = await PresentationFile.importPptx(await FileBlob.load(SIMPLE));
  const notesSource = await PresentationFile.importPptx(await FileBlob.load(OLD_DECK));
  const originalSlides = [...presentation.slides.items];
  const sourceSlide = originalSlides[1];
  const slides = [];
  for (let i = 0; i < 16; i += 1) slides.push(sourceSlide.duplicate());
  for (const slide of originalSlides) slide.delete();
  // duplicate() inserts each new copy immediately after the source, so the
  // collection order is the reverse of creation order once sources are removed.
  slides.reverse();
  slides.forEach((slide, i) => {
    slide.setIndex(i);
    clearLocalElements(slide);
    slide.background.fill = "#FFFFFF";
  });

  const logoBytes = await bytes(assets.logo);

  // 1 — title
  {
    const s = slides[0];
    s.images.add({ blob: logoBytes, contentType: "image/png", alt: "University of Groningen logo", fit: "contain", position: { left: 34, top: 20, width: 190, height: 52 } });
    addShape(s, "rect", 0, 92, W, 8, RED, RED, 0);
    addShape(s, "rect", 0, 118, W, 240, DARK, DARK, 0);
    addText(s, "Control of Mixed-Autonomy Traffic via\nAutonomous Vehicles with Lane-Changing Behavior", 66, 155, 1120, 170, { fontSize: 39, bold: true, color: "#FFFFFF", lineSpacing: 0.94, verticalAlignment: "middle" });
    addText(s, "Shuwei Pei, Muhammed O. Sayin, and Saeed Ahmed", 66, 468, 1000, 34, { fontSize: 21, bold: true });
    addText(s, "University of Groningen · Bilkent University", 66, 510, 900, 28, { fontSize: 17 });
    addText(s, "23rd IFAC World Congress · Busan · 23–28 August 2026", 66, 548, 1000, 28, { fontSize: 17 });
    addText(s, "Methodology: from reinforcement learning to an interpretable pair-aligned controller", 66, 627, 1120, 28, { fontSize: 14, color: MUTED });
  }

  // 2 — stop-and-go waves
  {
    const s = slides[1];
    await addChrome(s, "Stop-and-go waves become a two-dimensional control problem", 2, "Local longitudinal stabilization does not guarantee multi-lane system stability.", "Paper: Introduction and Problem Formulation; trajectory_log_allhuman44.png", logoBytes);
    addText(s, [{ runs: [{ run: "Stop-and-go waves are usually treated as a longitudinal-control problem.", textStyle: { bold: true } }], spaceAfter: 12 }, { runs: [{ run: "Human lane-changing adds a second disturbance channel.", textStyle: { bold: true, color: RED } }], spaceAfter: 12 }, ...bulletParagraphs(["Small braking disturbances propagate upstream.", "Lane changes abruptly modify leader–follower relationships.", "A controller confined to one lane cannot reject disturbances entering from another lane."])], 54, 155, 430, 380, { fontSize: 18, lineSpacing: 1.0 });
    await addImage(s, assets.allHuman, 520, 150, 695, 405, "Time-space trajectories showing stop-and-go waves", "contain");
    addText(s, "Lane-changing changes the active leader–follower graph.", 690, 552, 410, 26, { fontSize: 13, color: MUTED, alignment: "center" });
  }

  // 3 — methods comparison
  {
    const s = slides[2];
    await addChrome(s, "Existing methods solve different parts of the problem", 3, "The missing element is an interpretable controller that explicitly coordinates traffic across lanes.", "Paper: Introduction and cited ACC/CACC, MPC, Flow, and RL literature", logoBytes);
    addNativeTable(s, 70, 165, [250, 330, 550], [48, 104, 104, 104], [
      ["Paradigm", "Strength", "Limitation for this problem"],
      ["Rule-based control", "Interpretable and deployable", "Usually designed for longitudinal regulation within one lane"],
      ["Optimization-based control", "Handles models and constraints explicitly", "Requires repeated solution of a potentially expensive optimization problem"],
      ["Learning-based control", "Can adapt to complex HDV interactions", "Learned policies may be difficult to interpret and generalize"],
    ], { boldFirstColumn: true });
  }

  // 4 — questions and method chain
  {
    const s = slides[3];
    await addChrome(s, "The study follows three methodological questions", 4, "The sequence separates failure diagnosis, behavior discovery, and controller design.", "Paper: contribution and organization paragraphs", logoBytes);
    const qs = [
      ["Q1", "Can one RL-controlled AV stabilize two-lane traffic when HDVs change lanes?"],
      ["Q2", "What coordination structure appears when two AVs share information?"],
      ["Q3", "Can that structure be converted into an interpretable rule-based controller?"],
    ];
    qs.forEach((q, i) => {
      addText(s, q[0], 70, 155 + i * 72, 58, 38, { fontSize: 18, bold: true, color: "#FFFFFF", fill: RED, alignment: "center", verticalAlignment: "middle" });
      addText(s, q[1], 145, 151 + i * 72, 1020, 47, { fontSize: 20, bold: true, verticalAlignment: "middle" });
    });
    const labels = ["Single-agent RL", "Cooperative RL", "Behavioral interpretation", "PARC"];
    const nodes = labels.map((label, i) => addShape(s, "roundRect", 70 + i * 294, 445, 220, 70, i === 3 ? RED : LIGHT, i === 3 ? RED : LINE, 1.2, label));
    nodes.forEach((node, i) => addText(s, labels[i], node.frame.left + 8, node.frame.top + 12, node.frame.width - 16, 46, { fontSize: 16, bold: true, color: i === 3 ? "#FFFFFF" : TEXT, alignment: "center", verticalAlignment: "middle" }));
    for (let i = 0; i < nodes.length - 1; i += 1) s.shapes.connect(nodes[i], nodes[i + 1], { kind: "straight", fromSide: "right", toSide: "left", line: { style: "solid", fill: RED, width: 2 }, tail: { type: "arrow", width: "med", length: "med" } });
  }

  // 5 — testbed
  {
    const s = slides[4];
    await addChrome(s, "A controlled testbed isolates lane-changing disturbances", 5, "The ring removes boundary-flow effects and isolates cross-lane interactions.", "Paper: Simulation Setup; Flow/SUMO configurations; doublelaneringnetwork_2.png", logoBytes);
    await addImage(s, assets.ring, 70, 155, 445, 415, "Two-lane ring-road testbed", "contain");
    addText(s, bulletParagraphs(["Two-lane ring road, L = 250 m", "Ntotal = 44 vehicles", "SUMO microscopic simulation", "Flow reinforcement-learning interface", "IDM longitudinal dynamics", "Heterogeneous LC2013 lane-changing behavior", "Bounded AV acceleration commands"]), 560, 155, 620, 310, { fontSize: 17 });
    addText(s, [{ runs: [{ run: "HDVs:  ", textStyle: { bold: true, color: RED } }, "IDM + LC2013"] }, { runs: [{ run: "AVs:     ", textStyle: { bold: true, color: RED } }, "learned or rule-based longitudinal control"] }], 560, 480, 610, 80, { fontSize: 17 });
  }

  // 6 — MDP
  {
    const s = slides[5];
    await addChrome(s, "We formulate traffic control as a finite-horizon MDP", 6, "Observation, action, and reward design determine what coordination can be learned.", "Paper: Formulation as an RL Problem", logoBytes);
    addText(s, "𝓜 = (𝓢, 𝓐, 𝓣, r, ρ₀, γ, H)\n\nmaxπ J(π) = Eπ,𝓣 [ Σₜ₌₀ᴴ⁻¹ γᵗ r(sₜ, aₜ, sₜ₊₁) ]", 65, 190, 600, 240, { fontSize: 27, typeface: "Cambria Math", alignment: "center", verticalAlignment: "middle", lineSpacing: 1.15 });
    addShape(s, "rect", 690, 160, 1.5, 395, LINE, LINE, 0);
    addText(s, [{ runs: [{ run: "𝓢  ", textStyle: { bold: true, color: RED, typeface: "Cambria Math" } }, "local traffic and AV coordination states"] }, { runs: [{ run: "𝓐  ", textStyle: { bold: true, color: RED, typeface: "Cambria Math" } }, "AV acceleration and lane-change decisions"] }, { runs: [{ run: "𝓣  ", textStyle: { bold: true, color: RED, typeface: "Cambria Math" } }, "stochastic SUMO traffic evolution"] }, { runs: [{ run: "r   ", textStyle: { bold: true, color: RED, typeface: "Cambria Math" } }, "traffic efficiency and coordination objective"] }], 725, 180, 465, 330, { fontSize: 19, lineSpacing: 1.25 });
  }

  // 7 — single AV observation
  {
    const s = slides[6];
    await addChrome(s, "The single AV observes both lanes but acts locally", 7, "The reward is global, but the AV's physical authority remains local.", "Paper: Single-Agent Reinforcement Learning; singleagent_ring.py", logoBytes);
    addText(s, "oₜ = [vₜᴬⱽ/vₘₐₓ, zₜ⁽⁰⁾, zₜ⁽¹⁾] ∈ ℝ¹¹\n\nzₜ⁽ˡ⁾ = [𝟙{l=ℓₜ}, dᶠₜˡ/L, vᶠₜˡ/vₘₐₓ, dᵇₜˡ/L, vᵇₜˡ/vₘₐₓ]\n\naₜ = [aₜᵃᶜᶜ, aₜˡᶜ,ʳᵃʷ]\n\nrₜ = instantaneous average traffic speed", 55, 170, 690, 330, { fontSize: 22, typeface: "Cambria Math", lineSpacing: 1.12, verticalAlignment: "middle" });
    addShape(s, "rect", 805, 155, 350, 400, "#EFEFEF", LINE, 1);
    addShape(s, "rect", 978, 160, 3, 390, "#FFFFFF", "#FFFFFF", 0);
    for (let y = 175; y < 540; y += 38) addShape(s, "rect", 975, y, 9, 20, "#FFFFFF", "#FFFFFF", 0);
    const ego = addShape(s, "roundRect", 840, 330, 112, 52, BLUE, BLUE, 1);
    const f0 = addShape(s, "roundRect", 840, 205, 112, 44, "#FFFFFF", LINE, 1);
    const b0 = addShape(s, "roundRect", 840, 470, 112, 44, "#FFFFFF", LINE, 1);
    const f1 = addShape(s, "roundRect", 1005, 225, 112, 44, "#FFFFFF", LINE, 1);
    const b1 = addShape(s, "roundRect", 1005, 450, 112, 44, "#FFFFFF", LINE, 1);
    const vehicleLabels = [[ego, "ego AV", "#FFFFFF"], [f0, "front 0", TEXT], [b0, "back 0", TEXT], [f1, "front 1", TEXT], [b1, "back 1", TEXT]].map(([n, t, c]) => addText(s, t, n.frame.left + 5, n.frame.top + 7, n.frame.width - 10, n.frame.height - 14, { fontSize: 13, bold: t === "ego AV", color: c, alignment: "center", verticalAlignment: "middle" }));
    const observationLinks = [f0, b0, f1, b1].map((n) => s.shapes.connect(ego, n, { kind: "straight", line: { style: "solid", fill: RED, width: 1.3 }, tail: { type: "arrow", width: "sm", length: "sm" } }));
    observationLinks.forEach((link) => link.bringToFront());
    [ego, f0, b0, f1, b1, ...vehicleLabels].forEach((shape) => shape.bringToFront());
    addText(s, "lane 0", 840, 165, 112, 24, { fontSize: 12, bold: true, alignment: "center" });
    addText(s, "lane 1", 1005, 165, 112, 24, { fontSize: 12, bold: true, alignment: "center" });
  }

  // 8 — single-agent result
  {
    const s = slides[7];
    await addChrome(s, "The single-agent formulation produces a locally rational policy", 8, "Richer observation does not compensate for insufficient control authority.", "Paper: Single-Agent Results and Analysis; trajectory_log_1AVLC.png", logoBytes);
    addText(s, [{ runs: [{ run: "The AV learns to close the gap to its leader.", textStyle: { bold: true } }], spaceAfter: 14 }, ...numberedParagraphs(["A large gap invites lane-changing HDVs.", "Close-following removes one local gap.", "The AV reduces nearby merging—but cannot stabilize the adjacent lane."])], 55, 170, 430, 320, { fontSize: 19 });
    await addImage(s, assets.oneAv, 510, 150, 705, 410, "Single-AV time-space trajectories", "contain");
    addText(s, "Compression and expansion persist in both lane trajectories.", 690, 557, 390, 25, { fontSize: 13, color: MUTED, alignment: "center" });
  }

  // 9 — cooperative RL architecture
  {
    const s = slides[8];
    await addChrome(s, "Cooperative RL changes the information and action structure", 9, "The architecture adds direct authority over both lane boundaries.", "Paper: Cooperative Reinforcement Learning; singleagent_ring2AV.py", logoBytes);
    addText(s, "oₜⁱ = [vₜⁱ/vₘₐₓ, vₗₑₐdⁱ/vₘₐₓ, vfollowⁱ/vₘₐₓ, dleadⁱ/L, dfollowⁱ/L]\n\noₜ = [oₜ¹, oₜ², (xₜ¹−xₜ²)/L, (vₜ¹−vₜ²)/vₘₐₓ] ∈ ℝ¹²\n\naₜ = [a₁(t), a₂(t)]ᵀ ∈ [−0.5, 0.5]²", 55, 175, 720, 290, { fontSize: 22, typeface: "Cambria Math", lineSpacing: 1.15, verticalAlignment: "middle" });
    addText(s, bulletParagraphs(["Centralized observation", "Two longitudinal control inputs", "AVs remain in assigned lanes", "Pairwise position and velocity are explicit states"]), 825, 165, 380, 230, { fontSize: 18 });
    const a = addShape(s, "roundRect", 855, 445, 120, 62, BLUE, BLUE, 1);
    const b = addShape(s, "roundRect", 1060, 445, 120, 62, BLUE, BLUE, 1);
    addText(s, "AV 1", 865, 458, 100, 36, { fontSize: 17, bold: true, color: "#FFFFFF", alignment: "center", verticalAlignment: "middle" });
    addText(s, "AV 2", 1070, 458, 100, 36, { fontSize: 17, bold: true, color: "#FFFFFF", alignment: "center", verticalAlignment: "middle" });
    s.shapes.connect(a, b, { kind: "straight", fromSide: "right", toSide: "left", line: { style: "solid", fill: RED, width: 2 }, head: { type: "arrow", width: "sm", length: "sm" }, tail: { type: "arrow", width: "sm", length: "sm" } });
    addText(s, "partner state", 966, 417, 100, 24, { fontSize: 12, alignment: "center" });
  }

  // 10 — reward
  {
    const s = slides[9];
    await addChrome(s, "The cooperative reward explicitly shapes coordination", 10, "Pairing is reward-shaped learned behavior, not a reward-agnostic emergent phenomenon.", "Paper: Cooperative Reward Design; reviewer comments on reward shaping", logoBytes);
    addText(s, "rₜ = rpos,ₜ + rsync,ₜ + rspeed,ₜ + rsmooth,ₜ\n\nrpos,ₜ = −wpos |(xₜ¹−xₜ²)/L|\nrsync,ₜ = −wsync |(vₜ¹−vₜ²)/vₘₐₓ|\nrspeed,ₜ = wspeed ((vₜ¹+vₜ²)/(2vₘₐₓ))²\nrsmooth,ₜ = −wsmooth [(aₜ¹)²+(aₜ²)²]", 55, 160, 720, 390, { fontSize: 23, typeface: "Cambria Math", lineSpacing: 1.05, verticalAlignment: "middle" });
    const rewardRows = [["Position", "alignment"], ["Synchronization", "coherent motion"], ["Speed", "traffic efficiency"], ["Smoothness", "feasible acceleration"]];
    rewardRows.forEach((row, i) => {
      addText(s, row[0], 825, 185 + i * 78, 205, 40, { fontSize: 18, bold: true, color: RED, verticalAlignment: "middle" });
      addText(s, `→  ${row[1]}`, 1015, 185 + i * 78, 190, 40, { fontSize: 18, verticalAlignment: "middle" });
    });
  }

  // 11 — geometric mechanism
  {
    const s = slides[10];
    await addChrome(s, "The learned policy suggests a geometric control mechanism", 11, "The two-lane system behaves approximately as one virtual longitudinal lane.", "Paper: Cooperative RL Results; overview of paired controller", logoBytes);
    await addImage(s, assets.overview, 50, 160, 700, 400, "Overview of paired controller mechanism", "contain");
    addText(s, [{ runs: [{ run: "Aligning the AVs removes the staggered gaps used by HDVs to change lanes.", textStyle: { bold: true } }], spaceAfter: 16 }, ...numberedParagraphs(["The leading AV slows down.", "The trailing AV closes the cross-lane offset.", "The aligned AVs create a common moving boundary for both lanes."])], 780, 170, 420, 350, { fontSize: 19 });
  }

  // 12 — PARC controller
  {
    const s = slides[11];
    await addChrome(s, "PARC encodes the learned mechanism as a hybrid controller", 12, "The controller separates geometry, coherent motion, and efficient cruise.", "Paper: PARC strategy and Algorithm 1", logoBytes);
    addText(s, "dₚ = xₚ − x,     Δvₚ = vₚ − v\n\na = {  kpair dₚ,                     |dₚ| > dth\n        ksync (vₚ−v),              |vₚ−v| > δv\n        kv (v★−v),                  otherwise\n\na ← clip(a, −aₘₐₓ, aₘₐₓ)", 55, 155, 715, 405, { fontSize: 24, typeface: "Cambria Math", lineSpacing: 1.03, verticalAlignment: "middle" });
    const modes = [["Formation", "Regulate cross-lane position error."], ["Synchronization", "Remove relative velocity."], ["Cruise", "Track the shared equilibrium speed."]];
    modes.forEach((m, i) => {
      addShape(s, "rect", 825, 165 + i * 125, 365, 95, "#FAFAFA", LINE, 1);
      addText(s, m[0], 842, 177 + i * 125, 330, 28, { fontSize: 19, bold: true, color: RED });
      addText(s, m[1], 842, 210 + i * 125, 330, 36, { fontSize: 17 });
    });
  }

  // 13 — cruise speed
  {
    const s = slides[12];
    await addChrome(s, "The cruising speed follows the equilibrium traffic geometry", 13, "The cruise target respects effective-lane density—not only the road speed limit.", "Paper: cruise-speed derivation; paired_ring.py v_eq_max_function", logoBytes);
    addText(s, "Nℓ = Ntotal / 2 = 22\n\nseqᵐᵃˣ = (L − Nℓ smin) / (Nℓ − 1)\n\nseqᵐᵃˣ = (s₀ + v★τ) [1 − (v★/v₀)ᵞ]⁻¹ᐟ²", 55, 170, 680, 330, { fontSize: 28, typeface: "Cambria Math", alignment: "center", verticalAlignment: "middle", lineSpacing: 1.15 });
    addText(s, bulletParagraphs(["Road geometry determines feasible equilibrium spacing.", "The IDM relation maps spacing to the shared target speed v★.", "Both AVs track the same v★ after pairing."]), 790, 180, 410, 280, { fontSize: 19, lineSpacing: 1.05 });
    addShape(s, "rect", 790, 485, 410, 60, "#FAFAFA", LINE, 1);
    addText(s, "44 vehicles ÷ 2 lanes = 22 vehicles per effective lane", 805, 498, 380, 36, { fontSize: 16, bold: true, color: RED, alignment: "center", verticalAlignment: "middle" });
  }

  // 14 — validation
  {
    const s = slides[13];
    await addChrome(s, "The validation isolates the value of each methodological step", 14, "Coordination—not AV count alone—produces stable two-lane flow.", "Paper: Discussion and Comparison; trajectory_log_paired_controller.png", logoBytes);
    addNativeTable(s, 55, 165, [175, 465], [42, 64, 64, 64, 64, 64], [
      ["Method", "Scientific purpose"],
      ["All-HDV", "Demonstrate uncontrolled waves"],
      ["RL-1AV", "Test whether one learned AV is sufficient"],
      ["RL-Coop-2AV", "Test whether shared information creates useful coordination"],
      ["2AV-SLC", "Control for AV count without cross-lane coordination"],
      ["PARC", "Test the distilled pair-aligned mechanism"],
    ], { boldFirstColumn: true });
    await addImage(s, assets.paired, 730, 160, 485, 360, "Paired-controller time-space trajectories", "contain");
    addText(s, "After alignment: nearly parallel trajectories", 775, 525, 390, 28, { fontSize: 14, bold: true, color: RED, alignment: "center" });
  }

  // 15 — quantitative result
  {
    const s = slides[14];
    await addChrome(s, "RL identifies the structure; PARC executes it efficiently", 15, "RL discovers the coordination structure; PARC deploys it transparently.", "Paper: Discussion and Conclusion; avg_speed_comparison.png; reviewer comments", logoBytes);
    await addImage(s, assets.speed, 45, 155, 620, 350, "Average-speed comparison chart", "contain");
    addText(s, "4.441 m/s   →   4.768 m/s", 715, 170, 500, 55, { fontSize: 26, typeface: "Cambria Math", alignment: "center", verticalAlignment: "middle" });
    addText(s, "(4.768 − 4.441) / 4.441 × 100% = 7.4%", 715, 230, 500, 50, { fontSize: 22, bold: true, color: RED, typeface: "Cambria Math", alignment: "center", verticalAlignment: "middle" });
    addText(s, numberedParagraphs(["One AV cannot reject lane-changing disturbances across both lanes.", "Cooperative reward design identifies cross-lane alignment as a useful structure.", "PARC implements that structure transparently and improves stabilized speed."]), 735, 310, 455, 235, { fontSize: 17 });
    addText(s, "Current evidence: centralized information, two lanes, two AVs, and simulation-based validation.", 65, 560, 1120, 28, { fontSize: 12.5, color: MUTED, alignment: "center" });
  }

  // 16 — supplied official advertisement
  {
    const s = slides[15];
    s.background.fill = "#000000";
    await addImage(s, assets.ad, 0, 0, W, H, "Official IFAC 2029 Amsterdam advertisement", "cover");
  }

  // Preserve the exact speaker notes from the existing 16-slide deck.
  for (let i = 0; i < slides.length; i += 1) {
    slides[i].speakerNotes.setText(notesSource.slides.items[i].speakerNotes.text);
    slides[i].speakerNotes.setVisible(true);
  }

  const sourceNotes = [
    "Local source files only; no web assets were used.",
    "UoG logo: Docs/IFAC2026_PARC_DualFormat/assets/uog-logo.png (from UoG-Wide.pptx).",
    "Slide content and notes: Docs/IFAC2026_PARC_DualFormat/IFAC2026_PARC_Methodology.tex and existing notes-bearing PPTX.",
    "Final slide: IFAC29.Advertising.pdf, page 1, rasterized at 300 dpi.",
  ].join("\n");
  await fs.writeFile(path.join(TMP, "source-notes.txt"), sourceNotes, "utf8");

  const renderDir = path.join(TMP, "final-render");
  const layoutDir = path.join(TMP, "final-layout");
  await fs.mkdir(renderDir, { recursive: true });
  await fs.mkdir(layoutDir, { recursive: true });
  for (let i = 0; i < slides.length; i += 1) {
    const stem = `slide-${String(i + 1).padStart(2, "0")}`;
    const png = await presentation.export({ slide: slides[i], format: "png", scale: 1.5 });
    await fs.writeFile(path.join(renderDir, `${stem}.png`), new Uint8Array(await png.arrayBuffer()));
    const layout = await slides[i].export({ format: "layout" });
    await fs.writeFile(path.join(layoutDir, `${stem}.json`), await layout.text(), "utf8");
  }
  const montage = await presentation.export({ format: "png", montage: true, scale: 0.5 });
  await fs.writeFile(path.join(TMP, "final-montage.png"), new Uint8Array(await montage.arrayBuffer()));
  const inspection = await presentation.inspect({ kind: "slide,textbox,shape,image,notes,layout", include: "id,slide,name,title,textPreview,bbox,isPlaceholder,alt", maxChars: 80000 });
  await fs.writeFile(path.join(TMP, "final-inspect.ndjson"), inspection.ndjson, "utf8");
  const invalidNumbers = [];
  const scan = (value, keyPath = "root") => {
    if (typeof value === "number" && !Number.isFinite(value)) invalidNumbers.push(keyPath);
    else if (Array.isArray(value)) value.forEach((item, i) => scan(item, `${keyPath}[${i}]`));
    else if (value && typeof value === "object") for (const [key, item] of Object.entries(value)) scan(item, `${keyPath}.${key}`);
  };
  scan(presentation.toProto());
  await fs.writeFile(path.join(TMP, "invalid-numbers.txt"), invalidNumbers.join("\n"), "utf8");
  if (invalidNumbers.length) throw new Error(`Invalid numeric values:\n${invalidNumbers.join("\n")}`);
  const pptx = await PresentationFile.exportPptx(presentation);
  await pptx.save(OUT);
}

await main();
