import { expect, test } from "@playwright/test";

test.describe("Dashboard (mock mode)", () => {
  test.beforeEach(async ({ page }) => {
    await page.goto("/");
    await expect(page.getByRole("heading", { name: "Dashboard", level: 1 })).toBeVisible();
  });

  test("renders the simulation source, camera, battery and five joints", async ({ page }) => {
    // Source must be explicitly SIMULATION in mock mode.
    await expect(page.getByText("SIMULATION", { exact: true }).first()).toBeVisible();

    const camera = page.locator("section", {
      has: page.getByRole("heading", { name: "Camera · RGB" }),
    });
    await expect(camera).toBeVisible();
    await expect(camera.locator('img[alt="ArmPi camera"]')).toBeVisible();
    await expect(camera).toContainText("640×400");
    await expect(camera).toContainText("simulated");

    const auxCamera = page.locator("section", {
      has: page.getByRole("heading", { name: "Camera · Aux" }),
    });
    await expect(auxCamera).toBeVisible();
    await expect(auxCamera.locator('img[alt="ArmPi aux camera"]')).toBeVisible();
    await expect(auxCamera).toContainText("uvc:/dev/video4");
    await expect(auxCamera).toContainText("640×480");
    await expect(auxCamera).toContainText("simulated");

    const battery = page.locator("section", {
      has: page.getByRole("heading", { name: "Battery · 2S" }),
    });
    await expect(battery).toBeVisible();
    await expect(battery).toContainText("2S");
    await expect(battery).toContainText(/%/);
    await expect(battery).toContainText("simulated");

    const joints = page.locator("section", {
      has: page.getByRole("heading", { name: "Joint snapshot" }),
    });
    await expect(joints.locator("tbody tr")).toHaveCount(5);

    await expect(
      page.locator("section", { has: page.getByRole("heading", { name: "Safety gate" }) }),
    ).toContainText("0 blockers");
  });

  test("navigates the eight console pages", async ({ page }) => {
    for (const [link, title] of [
      ["Joint Control", "Joint Control"],
      ["Diagnostics", "Diagnostics"],
      ["Trajectory Inspector", "Trajectory Inspector"],
      ["Experiment Lab", "Experiment Lab"],
      ["Visual Workbench", "Visual Workbench"],
      ["Event Timeline", "Event Timeline"],
      ["Settings", "Settings"],
      ["Dashboard", "Dashboard"],
    ] as const) {
      await page.getByRole("link", { name: link }).click();
      await expect(page.getByRole("heading", { name: title, level: 1 })).toBeVisible();
    }
  });

  test("settings page reports mock mode and the verified mapping", async ({ page }) => {
    await page.getByRole("link", { name: "Settings" }).click();
    await expect(page.getByRole("heading", { name: "Settings", level: 1 })).toBeVisible();
    await expect(page.getByText("mock", { exact: true }).first()).toBeVisible();
    // The mapping was adopted from the 2026-11-19 stop survey; its version and
    // verified status are the contract the Settings page must show.
    await expect(
      page.getByText(/measured-2026-11-19/).first(),
    ).toBeVisible();
    await expect(page.getByText("verified", { exact: true }).first()).toBeVisible();
  });
});
