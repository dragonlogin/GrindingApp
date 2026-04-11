#include <QtTest>

#include "model_import/UrdfRobotImporter.h"

class TestUrdfRobotImporter : public QObject {
    Q_OBJECT

private:
    static std::string RobotPath(const char* relative)
    {
        return std::string(TEST_DATA_DIR) + "/" + relative;
    }

private slots:
    // 正常值：加载 IRB140.urdf 返回有效的 RobotDefinition
    void testImport_validUrdf_succeeds()
    {
        model_import::UrdfRobotImporter importer;
        auto result = importer.Import(RobotPath("IRB140/IRB140.urdf"));

        QVERIFY(result.ok());
        const auto& def = result.value();
        QCOMPARE(def.model.name, std::string("IRB140"));
        QVERIFY(!def.model.joints.empty());
        QVERIFY(!def.meshes.empty());
    }

    // 正常值：关节数量和网格数量符合预期（6 关节 + 7 网格）
    void testImport_irb140_hasCorrectJointAndMeshCount()
    {
        model_import::UrdfRobotImporter importer;
        auto result = importer.Import(RobotPath("IRB140/IRB140.urdf"));

        QVERIFY(result.ok());
        const auto& def = result.value();
        QCOMPARE(static_cast<int>(def.model.joints.size()), 6);
        QCOMPARE(static_cast<int>(def.meshes.size()), 7);
    }

    // 正常值：domain::Robot 的 source_path 等于输入路径
    void testImport_sourcePathPreserved()
    {
        model_import::UrdfRobotImporter importer;
        const std::string path = RobotPath("IRB140/IRB140.urdf");
        auto result = importer.Import(path);

        QVERIFY(result.ok());
        QCOMPARE(result.value().model.source_path, path);
    }

    // 正常值：每个 MeshRef 的 mesh_file 不为空
    void testImport_meshFilesNotEmpty()
    {
        model_import::UrdfRobotImporter importer;
        auto result = importer.Import(RobotPath("IRB140/IRB140.urdf"));

        QVERIFY(result.ok());
        for (const auto& mesh : result.value().meshes)
            QVERIFY(!mesh.mesh_file.empty());
    }

    // 边界值：路径不存在 → 返回 Fail
    void testImport_nonexistentFile_returnsFail()
    {
        model_import::UrdfRobotImporter importer;
        auto result = importer.Import("/nonexistent/path/robot.urdf");

        QVERIFY(!result.ok());
    }

    // 边界值：空路径 → 返回 Fail
    void testImport_emptyPath_returnsFail()
    {
        model_import::UrdfRobotImporter importer;
        auto result = importer.Import("");

        QVERIFY(!result.ok());
    }
};

QTEST_MAIN(TestUrdfRobotImporter)
#include "TestUrdfRobotImporter.moc"
