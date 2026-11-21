import { expect, test } from "@playwright/test";

// Dedicated pose-scope mock backend:
//   E2E_POSE=1 npm run test:e2e
// It never touches the robot: ARMPI_MODE=mock, no gateway, no dispatch.
test.describe("Pose Takeover (mock pose scope)", () => {
  test("preset → preview → confirm → execute completes", async ({ page }) => {
    await page.goto("/#/joints");
    await expect(
      page.getByRole("heading", { name: "Pose Takeover", level: 1 }),
    ).toBeVisible();

    await page.getByRole("button", { name: "申请控制权" }).click();
    await expect(page.getByText("lease held", { exact: true })).toBeVisible();

    // The pose library comes from /api/motion/presets.
    const library = page.locator("section", {
      has: page.getByRole("heading", { name: "Pose library" }),
    });
    await expect(library).toContainText("低位抓取姿态");
    await page.getByRole("button", { name: /低位抓取姿态/ }).click();

    await page.getByRole("button", { name: "预览姿态" }).click();
    const preview = page.locator("section", {
      has: page.getByRole("heading", { name: "Pose preview" }),
    });
    await expect(preview.getByText("VALID", { exact: true })).toBeVisible();
    await expect(preview).toContainText("Target TCP");
    await expect(preview).toContainText("Table clearance");

    await page.getByRole("button", { name: "执行姿态（需确认）" }).click();
    const dialog = page.getByRole("dialog", { name: "确认姿态命令" });
    await expect(dialog).toBeVisible();
    await dialog.getByRole("button", { name: "确认执行" }).click();

    const command = page.locator("section", {
      has: page.getByRole("heading", { name: /Command status/ }),
    });
    await expect(command).toBeVisible();
    await expect(command.getByText("Completed", { exact: true }).first()).toBeVisible({
      timeout: 15_000,
    });

    // Release the lease so the next run starts clean.
    await page.getByRole("link", { name: "Dashboard" }).click();
    await page.getByRole("button", { name: "释放控制权" }).click();
    await expect(page.getByText("当前没有控制权")).toBeVisible();
  });

  test("gripper preview is valid and timed", async ({ page }) => {
    await page.goto("/#/joints");
    await expect(
      page.getByRole("heading", { name: "Pose Takeover", level: 1 }),
    ).toBeVisible();

    // Preview needs no control lease; only execute does.
    await page.getByRole("button", { name: "预览闭合" }).click();
    const preview = page.locator("section", {
      has: page.getByRole("heading", { name: "Gripper preview" }),
    });
    await expect(preview.getByText("VALID", { exact: true })).toBeVisible();
    await expect(preview).toContainText("r_joint");
    await expect(preview).toContainText("timed");
  });
});
