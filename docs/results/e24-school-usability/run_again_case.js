// (e24, simulator only) The audit's B2: per-robot lists, a practice run, the page RELOADED, then "Run again".
// Which order does the second run drive? Mission Control must be running in practice mode at BASE.
//   node run_again_case.js BASE RUNS_DIR        (RUNS_DIR: Mission Control's --dir + /runs)
// Prints one JSON line: the order typed, and the second run's own record of what it was asked (visit_order.json).
// Works on the page before e24 (clicks on the map) and on e24's. Changelog: 2026-11-24 first version.
const { chromium } = require('playwright');
const fs = require('fs'), path = require('path');
const BASE = process.argv[2], RUNS = process.argv[3];
const WANT = { Alpha: ['P01', 'P02'], Bravo: ['P12', 'P14'] };
const newest = () => fs.readdirSync(RUNS).filter(n => n.includes('-visit-')).sort().pop();
(async () => {
  const browser = await chromium.launch();
  const page = await (await browser.newContext({ viewport: { width: 1366, height: 768 } })).newPage();
  page.on('dialog', d => d.accept());
  await page.goto(BASE + '/', { waitUntil: 'domcontentloaded' }); await page.waitForTimeout(2500);
  await page.click('button[data-k=visit]'); await page.waitForTimeout(1500);
  await page.click('#vpeach'); await page.waitForTimeout(600);
  for (const p of WANT.Alpha) { await page.click(`g.stk[data-p=${p}]`, { force: true }); await page.waitForTimeout(250); }
  await page.click('button[data-who=Bravo]'); await page.waitForTimeout(300);
  for (const p of WANT.Bravo) { await page.click(`g.stk[data-p=${p}]`, { force: true }); await page.waitForTimeout(250); }
  await page.click('#start');
  const first = await (async () => { for (let i = 0; i < 20; i++) { await page.waitForTimeout(1000); try { const n = newest(); if (n) return n; } catch (e) {} } return null; })();
  let ended = false;
  for (let i = 0; i < 200 && !ended; i++) { await page.waitForTimeout(3000); ended = !!(await page.$('#again')); }
  await page.reload({ waitUntil: 'domcontentloaded' }); await page.waitForTimeout(3500);
  const label = await page.evaluate(() => (document.getElementById('again') || {}).innerText || null);
  await page.click('#again');
  let second = null;
  for (let i = 0; i < 40 && !second; i++) { await page.waitForTimeout(1500); const n = newest(); if (n && n !== first && fs.existsSync(path.join(RUNS, n, 'visit_order.json'))) second = n; }
  let asked = null, orders = null;
  let note = '';
  if (second) { const vo = JSON.parse(fs.readFileSync(path.join(RUNS, second, 'visit_order.json'))); asked = vo.asked; orders = vo.orders; }
  else { const n = newest(); note = 'the second run wrote no visit_order.json (no order was sent: the points shared out)'; second = n !== first ? n : null; }
  await page.waitForTimeout(1000);
  await page.click('#stoptop').catch(() => {});
  await page.waitForTimeout(8000);
  const n2 = orders ? Object.values(orders).reduce((a, v) => a + v.length, 0) : null;
  console.log(JSON.stringify({ typed: WANT, first_run_ended: ended, run_again_button: label, second_run: second, second_run_asked: asked, second_run_points: n2, same_lists: asked === 'Alpha:P01,P02;Bravo:P12,P14', note }));
  await browser.close();
})();
