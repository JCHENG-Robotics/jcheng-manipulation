import { expect, test } from "@playwright/test";

// Dedicated pose-scope mock backend:
//   E2E_POSE=1 npm run test:e2e
// The task uses the simulated locate point; no camera and no robot.
test.describe("Task Takeover (mock pose scope)", () => {
  test("prepare → locate/plan → run → verify → put-back", async ({ page }) => {
    test.setTimeout(120_000);
    await page.goto("/#/task");
    await expect(
      page.getByRole("heading", { name: "Task Takeover", level: 1 }),
    ).toBeVisible();

    await page.getByRole("button", { name: "准备（移动到 view）" }).click();
    await expect(page.getByText(/已到 view/)).toBeVisible({ timeout: 25_000 });

    await page.getByRole("button", { name: "定位并规划" }).click();
    const planning = page.locator("section", {
      has: page.getByRole("heading", { name: "准备与定位" }),
    });
    await expect(planning.getByText("PLAN VALID", { exact: true })).toBeVisible({
      timeout: 20_000,
    });
    await expect(planning).toContainText("Cube point");
    await expect(planning).toContainText("Fit xy error");

    const steps = page.locator("section", {
      has: page.getByRole("heading", { name: "Steps" }),
    });
    await expect(steps).toContainText("open_gripper");
    await expect(steps).toContainText("grasp");
    await expect(steps).toContainText("grip_verify");
    await expect(steps).toContainText("return_view");

    await page.getByRole("button", { name: "运行任务（需确认）" }).click();
    const dialog = page.getByRole("dialog", { name: "确认执行任务" });
    await expect(dialog).toBeVisible();
    await dialog.getByRole("button", { name: /确认执行/ }).click();

    const verifyCard = page.locator("section", {
      has: page.getByRole("heading", { name: "夹持验证" }),
    });
    await expect(verifyCard.getByText("HELD", { exact: true })).toBeVisible({
      timeout: 90_000,
    });

    // The put-back continues after the verify; wait for the final step.
    const returnRow = steps.getByRole("row", { name: /return_view/ });
    await expect(returnRow.getByText("completed", { exact: true })).toBeVisible({
      timeout: 60_000,
    });
    const statuses = await steps.getByText("completed", { exact: true }).count();
    expect(statuses).toBeGreaterThan(8);

    await page.getByRole("link", { name: "Dashboard" }).click();
    await page.getByRole("button", { name: "释放控制权" }).click();
    await expect(page.getByText("当前没有控制权")).toBeVisible();
  });
});
