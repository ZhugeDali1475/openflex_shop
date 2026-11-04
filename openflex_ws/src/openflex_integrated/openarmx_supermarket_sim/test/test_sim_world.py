import pathlib

from openarmx_supermarket_sim.sim_world import CompetitionWorld


CONFIG = pathlib.Path(__file__).resolve().parents[1] / "config" / "world.yaml"


def test_seed_reproduces_the_same_match():
    first = CompetitionWorld(str(CONFIG), 41)
    second = CompetitionWorld(str(CONFIG), 41)
    assert [entry.sku for entry in first.order] == [entry.sku for entry in second.order]
    assert [work.object_id for work in first.restock] == [
        work.object_id for work in second.restock
    ]


def test_complete_match_scores_300_points():
    world = CompetitionWorld(str(CONFIG), 72)
    for entry in world.order:
        assert world.pick(entry.sku, entry.source_station)
        assert world.place(entry.sku, entry.destination_station)
        assert world.verify(entry.sku, entry.destination_station)
    for work in [*world.restock, *world.misplaced]:
        assert world.pick(work.object_id, work.source_station)
        assert world.place(work.object_id, work.destination_station)
        assert world.verify(
            work.object_id, work.destination_station, work.destination_slot
        )
    assert world.phase_verified == {1: 2, 2: 2, 3: 2}
    assert world.raw_points == 300
    assert world.score == 300


def test_penalties_and_wrong_placement_are_enforced():
    world = CompetitionWorld(str(CONFIG), 9)
    entry = world.order[0]
    assert world.pick(entry.sku, entry.source_station)
    assert world.place(entry.sku, "wrong_station")
    assert not world.verify(entry.sku, "wrong_station")
    world.record_collision()
    world.record_out_of_bounds()
    world.drops += 1
    assert world.penalty_points == 20


def test_only_selected_anomalies_are_visible():
    world = CompetitionWorld(str(CONFIG), 17)
    selected = world.restock[0]
    visible = world.work_visible(selected.phase, selected.visible_from)
    assert selected in visible
    assert all(work in world.restock for work in visible)
