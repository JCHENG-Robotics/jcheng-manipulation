import { expect, test } from "@playwright/test";

test.describe("Experiment Lab (offline, mock backend)", () => {
  test("loads the simulated paired demo and reports honest analysis limits", async ({
    page,
  }) => {
    await page.goto("/");
    await page.getByRole("link", { name: "Experiment Lab" }).click();
    await expect(page.getByRole("heading", { name: "Experiment Lab", level: 1 })).toBeVisible();

    const before = await page.request.get("/api/events?limit=1");
    const beforeBody = (await before.json()) as { items: Array<{ seq: number }> };
    const lastSeq = beforeBody.items[0]?.seq ?? 0;

    await page.getByRole("button", { name: "加载示例实验" }).click();
    await expect(page.getByText("G3 demo - simulated paired grasp").first()).toBeVisible({
      timeout: 15_000,
    });
    await expect(page.getByRole("heading", { name: "Paired analysis" })).toBeVisible();

    const improvement = page
      .locator(".metric-row-4 .metric")
      .filter({ hasText: "Mean improvement" });
    await expect(improvement).toContainText("1.161");
    await expect(improvement).toContainText("mm");

    const trials = page.locator("section", {
      has: page.getByRole("heading", { name: "Recorded trials" }),
    });
    await expect(trials.locator("tbody tr")).toHaveCount(32);

    // The demo is simulated and the binary endpoint is explicitly underpowered.
    await expect(page.getByText("simulated", { exact: true }).first()).toBeVisible();
    await expect(
      page.getByText("do not claim a significant success-rate improvement", { exact: false }),
    ).toBeVisible();
    await expect(page.getByRole("link", { name: "Trials CSV" })).toBeVisible();
    await expect(page.getByRole("link", { name: "Metadata JSON" })).toBeVisible();

    const after = await page.request.get(`/api/events?after_seq=${lastSeq}`);
    const afterBody = (await after.json()) as { items: Array<{ stage: string }> };
    const stages = afterBody.items.map((event) => event.stage);
    expect(stages).not.toContain("Dispatched");
    expect(stages).not.toContain("Requested");
  });
});
